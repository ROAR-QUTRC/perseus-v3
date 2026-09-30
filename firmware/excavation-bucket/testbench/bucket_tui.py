#!/usr/bin/env python3
"""Bucket CAN console: live hi-can monitor on top, bucket commands below.

Every address comes from hi_can_address.hpp, parsed at startup with libclang
(the libclang wheel bundles it; `clang` on PATH supplies the builtin headers).

    python3 -m venv .venv
    .venv/bin/pip install -r requirements.txt
    .venv/bin/python bucket_tui.py --iface can0
"""

import argparse
import asyncio
import ctypes
import socket
import struct
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import clang.cindex as ci
from textual import events
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import (
    Button,
    Checkbox,
    DataTable,
    Footer,
    Input,
    Label,
    RichLog,
    Select,
    Static,
)

HEADER = (
    Path(__file__).resolve().parents[3]
    / "software/shared/hi-can/include/hi_can_address.hpp"
)

# Names the console needs from hi-can. A rename there stops it at startup.
CONTROLLER = "excavation::bucket::controller"
BANK_GROUP = f"{CONTROLLER}::bank_group"
BANK_PARAMETER = f"{CONTROLLER}::bank_parameter"
HOME_PARAMETER = "HOME"  # not in hi-can yet; the Home button does nothing until it is
ZERO_PARAMETER = "SET_ZERO_POS"  # empty payload, as ROS's zero_axis() sends it
FAULT_PARAMETER = "GET_FAULT"

# Payload formats aren't in hi-can; these follow the bucket firmware.
POSITION_UNITS_PER_DEGREE = 10
FULL_DUTY = 32767  # SET_SPEED +/-32767 = 100 %
STOP_NUDGE_PERCENT = (
    2  # just outside the 1 % deadband, so Stop also cancels position mode
)

JOG_PERIOD_S = 0.05  # the firmware stops a bank ~600 ms after its last SET_SPEED (200 ms x hi-can's 3 misses)
KEY_REPEAT_DELAY_S = (
    0.75  # outlasts auto-repeat delay (X11 default 660 ms, Wayland desktops 500-600 ms)
)
KEY_REPEAT_GAP_S = 0.15  # once repeating, a gap this long means the key was released
REFRESH_S = 0.1
ZERO_CONFIRM_S = 3.0  # Zero needs a second press within this long
FAULT_STALE_S = 2.0  # no GET_FAULT rate is agreed yet; older than this shows as stale
RTR_REPLY_TIMEOUT_S = 1.0

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x7FF
CAN_FRAME = struct.Struct("=IB3x8s")


# --- hi-can -----------------------------------------------------------------


@dataclass
class Namespace:
    values: dict = field(default_factory=dict)
    enums: dict = field(default_factory=dict)
    children: dict = field(default_factory=dict)


@dataclass
class Group:
    name: str
    parameters: dict  # id -> (name, qualified name)


@dataclass
class Device:
    name: str
    groups: dict  # id -> Group


def _libclang():
    lib = ci.conf.lib
    lib.clang_Cursor_Evaluate.argtypes = [ci.Cursor]
    lib.clang_Cursor_Evaluate.restype = ctypes.c_void_p
    lib.clang_EvalResult_getKind.argtypes = [ctypes.c_void_p]
    lib.clang_EvalResult_getKind.restype = ctypes.c_int
    lib.clang_EvalResult_getAsLongLong.argtypes = [ctypes.c_void_p]
    lib.clang_EvalResult_getAsLongLong.restype = ctypes.c_longlong
    lib.clang_EvalResult_dispose.argtypes = [ctypes.c_void_p]
    return lib


def _evaluate(lib, cursor):
    result = lib.clang_Cursor_Evaluate(cursor)
    if not result:
        return None
    try:
        return (
            lib.clang_EvalResult_getAsLongLong(result)
            if lib.clang_EvalResult_getKind(result) == 1
            else None
        )
    finally:
        lib.clang_EvalResult_dispose(result)


def _parse(header: Path) -> Namespace:
    if not header.is_file():
        raise RuntimeError(f"{header} not found")
    try:
        resource_dir = subprocess.run(
            ["clang", "-print-resource-dir"], capture_output=True, text=True
        ).stdout.strip()
    except FileNotFoundError:
        resource_dir = ""
    args = ["-x", "c++", "-std=c++20"] + (
        ["-resource-dir", resource_dir] if resource_dir else []
    )
    tu = ci.Index.create().parse(str(header), args=args)
    errors = [d.spelling for d in tu.diagnostics if d.severity >= ci.Diagnostic.Error]
    if errors:
        hint = "" if resource_dir else " (no clang on PATH for its builtin headers)"
        raise RuntimeError(f"{header}: " + "; ".join(errors) + hint)

    lib = _libclang()
    root = Namespace()

    def collect(cursor, ns):
        for child in cursor.get_children():
            if child.location.file is None or child.location.file.name != str(header):
                continue
            if child.kind == ci.CursorKind.NAMESPACE:
                collect(child, ns.children.setdefault(child.spelling, Namespace()))
            elif child.kind == ci.CursorKind.VAR_DECL:
                value = _evaluate(lib, child)
                if value is not None:
                    ns.values[child.spelling] = value
            elif child.kind == ci.CursorKind.ENUM_DECL:
                ns.enums[child.spelling] = {
                    e.spelling: e.enum_value
                    for e in child.get_children()
                    if e.kind == ci.CursorKind.ENUM_CONSTANT_DECL
                }

    collect(tu.cursor, root)
    return root.children["hi_can"].children["addressing"]


class HiCan:
    LEVELS = ("SYSTEM", "SUBSYSTEM", "DEVICE", "GROUP", "PARAM")

    def __init__(self, header: Path):
        self.root = _parse(header.resolve())
        values = self.root.values
        self.layout = [
            (values[f"{level}_ADDRESS_POS"], values[f"{level}_ADDRESS_BITS"])
            for level in self.LEVELS
        ]
        self.masks = {level: values[f"{level}_MASK"] for level in self.LEVELS[:4]}
        self.mask_all = values["MASK_ALL"]
        self.names = {}  # (system,) or (system, subsystem) -> dotted name
        self.devices = {}  # (system, subsystem, device) -> Device
        self.filters = [("all traffic", 0, 0)]  # (label, address, mask)
        self.parameters = set()  # qualified names
        self.common_groups = {}
        self._build()

    def pack(self, *ids) -> int:
        raw = 0
        for (pos, bits), value in zip(self.layout, ids):
            raw |= (value & ((1 << bits) - 1)) << pos
        return raw

    def unpack(self, raw: int) -> tuple:
        return tuple((raw >> pos) & ((1 << bits) - 1) for pos, bits in self.layout)

    def namespace(self, path: str) -> Namespace:
        ns = self.root
        for part in path.split("::"):
            ns = ns.children[part]
        return ns

    def enum(self, path: str) -> dict:
        scope, _, name = path.rpartition("::")
        return self.namespace(scope).enums[name]

    def device_ids(self, path: str) -> tuple:
        system, subsystem, device = path.split("::")
        system_ns = self.root.children[system]
        subsystem_ns = system_ns.children[subsystem]
        return (
            system_ns.values["SYSTEM_ID"],
            subsystem_ns.values["SUBSYSTEM_ID"],
            subsystem_ns.children[device].values["DEVICE_ID"],
        )

    def describe(self, raw: int) -> tuple:
        """(device, parameter, qualified parameter name or None) for an address."""
        system, subsystem, device_id, group_id, parameter_id = self.unpack(raw)
        device = self.devices.get((system, subsystem, device_id))
        if device:
            device_name = device.name
        else:
            parent = self.names.get((system, subsystem)) or self.names.get(
                (system,), f"sys{system}"
            )
            device_name = f"{parent}.{'' if (system, subsystem) in self.names else f'sub{subsystem}.'}dev{device_id}"
        group = (
            device.groups.get(group_id) if device else None
        ) or self.common_groups.get(group_id)
        if group is None:
            return device_name, f"g{group_id:02X} p{parameter_id:02X}", None
        name, qualified = group.parameters.get(
            parameter_id, (f"p{parameter_id:02X}", None)
        )
        return device_name, f"{group.name} {name}", qualified

    def parameter_list(self) -> list:
        """(label, address) for every parameter hi-can defines, sorted by address.
        Devices without their own groups get the common status group."""
        entries = []
        for ids, device in self.devices.items():
            short_name = device.name.split(".", 1)[
                -1
            ]  # the system is in the address anyway
            for group_id, group in (device.groups or self.common_groups).items():
                for parameter_id, (name, _) in group.parameters.items():
                    address = self.pack(*ids, group_id, parameter_id)
                    entries.append((f"{short_name} {group.name} {name}", address))
        return sorted(entries, key=lambda entry: entry[1])

    def _params(self, members: dict, scope: str) -> dict:
        params = {}
        for name, value in members.items():
            params[value] = (name, f"{scope}::{name}")
            self.parameters.add(f"{scope}::{name}")
        return params

    def _groups(self, ns: Namespace, scope: str) -> dict:
        groups = {}
        for enum_name, members in ns.enums.items():
            if enum_name.endswith("_group"):
                param_enum = enum_name.removesuffix("_group") + "_parameter"
                params = self._params(
                    ns.enums.get(param_enum, {}), f"{scope}::{param_enum}"
                )
                for name, group_id in members.items():
                    groups[group_id] = Group(name, params)
        for name, child in ns.children.items():
            if "GROUP_ID" in child.values:
                params = self._params(
                    child.enums.get("parameter", {}), f"{scope}::{name}::parameter"
                )
                groups[child.values["GROUP_ID"]] = Group(name, params)
        return groups

    def _add_device(self, name: str, ids: tuple, groups: dict):
        self.devices[ids] = Device(name, groups)
        self.filters.append((name, self.pack(*ids), self.masks["DEVICE"]))
        for group_id, group in sorted(groups.items()):
            self.filters.append(
                (f"{name}.{group.name}", self.pack(*ids, group_id), self.masks["GROUP"])
            )

    def _build(self):
        status = self.root.children.get("status")
        if status and "GROUP_ID" in status.values:
            self.common_groups[status.values["GROUP_ID"]] = Group(
                "status",
                self._params(status.enums.get("parameter", {}), "status::parameter"),
            )

        for system_name, system_ns in self.root.children.items():
            if system_name == "legacy" or "SYSTEM_ID" not in system_ns.values:
                continue
            system = system_ns.values["SYSTEM_ID"]
            self.names[(system,)] = system_name
            self.filters.append((system_name, self.pack(system), self.masks["SYSTEM"]))
            for subsystem_name, subsystem_ns in system_ns.children.items():
                if "SUBSYSTEM_ID" not in subsystem_ns.values:
                    continue
                subsystem = subsystem_ns.values["SUBSYSTEM_ID"]
                label = f"{system_name}.{subsystem_name}"
                self.names[(system, subsystem)] = label
                self.filters.append(
                    (label, self.pack(system, subsystem), self.masks["SUBSYSTEM"])
                )
                for name, device in subsystem_ns.enums.get("device", {}).items():
                    self._add_device(f"{label}.{name}", (system, subsystem, device), {})
                for name, device_ns in subsystem_ns.children.items():
                    if "DEVICE_ID" in device_ns.values:
                        ids = (system, subsystem, device_ns.values["DEVICE_ID"])
                        groups = self._groups(
                            device_ns, f"{system_name}::{subsystem_name}::{name}"
                        )
                        self._add_device(f"{label}.{name}", ids, groups)


# --- payloads ---------------------------------------------------------------


def _degrees(data: bytes) -> str:
    return (
        f"{struct.unpack('<h', data)[0] / POSITION_UNITS_PER_DEGREE:.1f}°"
        if len(data) == 2
        else ""
    )


def _percent(data: bytes) -> str:
    return (
        f"{struct.unpack('<h', data)[0] * 100 / FULL_DUTY:+.0f} %"
        if len(data) == 2
        else ""
    )


def _amps(data: bytes) -> str:
    # Sent in mA.
    return f"{struct.unpack('<H', data)[0] / 1000:.3f} A" if len(data) == 2 else ""


def _fault(data: bytes) -> str:
    # Planned: 1-byte fault flags, 0 = healthy. Bit names aren't in hi-can yet, so show hex.
    if len(data) != 1:
        return ""
    return "ok" if data[0] == 0 else f"FAULT 0x{data[0]:02X}"


DECODERS = {
    f"{BANK_PARAMETER}::GET_POSITION": _degrees,
    f"{BANK_PARAMETER}::SET_POSITION": _degrees,
    f"{CONTROLLER}::encoder_parameter::GET_ANGLE": _degrees,
    f"{BANK_PARAMETER}::SET_SPEED": _percent,
    f"{BANK_PARAMETER}::GET_CURRENT": _amps,
    f"{BANK_PARAMETER}::{FAULT_PARAMETER}": _fault,
}


# --- bus --------------------------------------------------------------------


class CanBus:
    def __init__(self, iface: str):
        self.rx = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.rx.bind((iface,))
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.tx.bind((iface,))
        self.tx.setblocking(False)

    def send(self, address: int, data: bytes = b"", rtr: bool = False, dlc: int = 0):
        """An RTR frame carries no data; `dlc` is the length it asks for."""
        can_id = address | CAN_EFF_FLAG | (CAN_RTR_FLAG if rtr else 0)
        length = dlc if rtr else len(data)
        self.tx.send(
            CAN_FRAME.pack(can_id, length, b"" if rtr else data.ljust(8, b"\0"))
        )

    def receive(self):
        """Yields (id, extended, rtr, data) for every frame waiting."""
        while True:
            try:
                frame = self.rx.recv(CAN_FRAME.size)
            except BlockingIOError:
                return
            can_id, length, data = CAN_FRAME.unpack(frame)
            if can_id & CAN_ERR_FLAG:
                continue
            extended = bool(can_id & CAN_EFF_FLAG)
            yield (
                can_id & (CAN_EFF_MASK if extended else CAN_SFF_MASK),
                extended,
                bool(can_id & CAN_RTR_FLAG),
                data[:length],
            )


class Bucket:
    def __init__(self, hican: HiCan):
        self.hican = hican
        self.device = hican.device_ids(CONTROLLER)
        self.banks = hican.enum(BANK_GROUP)
        self.parameters = hican.enum(BANK_PARAMETER)
        for required in ("SET_SPEED", "SET_POSITION"):
            if required not in self.parameters:
                raise KeyError(f"{BANK_PARAMETER}::{required}")

    def address(self, bank: str, parameter: str) -> int:
        return self.hican.pack(
            *self.device, self.banks[bank], self.parameters[parameter]
        )


# --- UI ---------------------------------------------------------------------


@dataclass
class Seen:
    device: str
    parameter: str
    decode: object
    data: bytes = b""
    rtr: bool = False
    count: int = 0
    times: deque = field(default_factory=deque)
    shown: tuple = ()

    def data_text(self) -> str:
        return "R" if self.rtr else " ".join(f"{b:02X}" for b in self.data)

    def value_text(self) -> str:
        return self.decode(self.data) if self.decode and not self.rtr else ""


class JogPad(Static, can_focus=True):
    def on_key(self, event: events.Key) -> None:
        if event.key in ("up", "down"):
            self.app.jog(1 if event.key == "up" else -1)
        elif event.key in ("left", "right"):
            self.app.step_bank(-1 if event.key == "left" else 1)
        elif event.key == "space":
            self.app.stop()
        else:
            return
        event.stop()
        event.prevent_default()


class BucketConsole(App):
    TITLE = "Bucket CAN console"
    CSS = """
    #top { height: 1fr; }
    #monitor { width: 1fr; border: round $primary; }
    #manual { width: 52; border: round $accent; }
    #manual_param, #manual Input { width: 1fr; }
    #manual .row Label { width: 7; }
    #manual_preview, #manual_reply { height: auto; padding: 0 1; }
    #filterbar, .row { height: auto; }
    #filter { width: 48; }
    #raw_filter { width: 34; }
    #rate { padding: 1 1; }
    DataTable, RichLog { height: 1fr; }
    RichLog { display: none; }
    #commands { height: auto; border: round $secondary; }
    .row Label { padding: 1 1 0 1; }
    #bank { width: 18; }
    #speed, #position { width: 12; }
    #jog { height: 3; border: tall $panel; padding: 0 1; }
    #jog:focus { border: tall $success; }
    #faults, #tx { height: 1; padding: 0 1; }
    """
    BINDINGS = [
        Binding("q", "quit", "Quit"),
        Binding("escape", "focus_jog", "Jog"),
        Binding("l", "toggle_view", "Table/log"),
        Binding("p", "toggle_pause", "Pause"),
        Binding("c", "clear", "Clear"),
    ]

    def __init__(self, hican: HiCan, bucket: Bucket, bus: CanBus, iface: str):
        super().__init__()
        self.hican, self.bucket, self.bus, self.iface = hican, bucket, bus, iface
        labels = [label for label, _, _ in hican.filters]
        default = CONTROLLER.replace("::", ".")
        self.filter_index = labels.index(default) if default in labels else 0
        self.filter = hican.filters[self.filter_index]
        self.seen = {}
        self.pending_log = deque(maxlen=500)
        self.paused = False
        self.show_log = False
        self.t0 = time.monotonic()
        self.bank = next(iter(bucket.banks))
        self.jog_dir = 0
        self.jog_until = 0.0
        self.last_error = ""
        self.zero_armed_bank = None
        self.zero_disarm_timer = None
        # GET_FAULT is tracked whatever the monitor filter shows: address -> bank, bank -> (byte, time)
        self.fault_banks = (
            {bucket.address(bank, FAULT_PARAMETER): bank for bank in bucket.banks}
            if FAULT_PARAMETER in bucket.parameters
            else {}
        )
        self.faults = {}
        self.manual_params = hican.parameter_list()
        self.rtr_request = None  # (address, time sent) while waiting for an RTR reply
        self.timers = []

    def compose(self) -> ComposeResult:
        with Horizontal(id="top"):
            with Vertical(id="monitor"):
                with Horizontal(id="filterbar"):
                    options = [
                        (label, i) for i, (label, _, _) in enumerate(self.hican.filters)
                    ]
                    yield Select(
                        options, id="filter", allow_blank=False, value=self.filter_index
                    )
                    yield Input(placeholder="raw ID or ID/MASK (hex)", id="raw_filter")
                    yield Label("", id="rate")
                yield DataTable(show_cursor=False)
                yield RichLog(max_lines=2000)
            with Vertical(id="manual"):
                yield Select(
                    [(label, i) for i, (label, _) in enumerate(self.manual_params)],
                    id="manual_param",
                    prompt="parameter (type to search)",
                )
                with Horizontal(classes="row"):
                    yield Label("ID")
                    yield Input(placeholder="hex, e.g. 02000004", id="manual_id")
                with Horizontal(classes="row"):
                    yield Label("Data")
                    yield Input(placeholder="hex bytes, e.g. 48 0D", id="manual_data")
                with Horizontal(classes="row"):
                    yield Checkbox("RTR", id="manual_rtr")
                    yield Button("Send", id="manual_send", variant="primary")
                yield Static("", id="manual_preview")
                yield Static("", id="manual_reply")
        with Vertical(id="commands"):
            with Horizontal(classes="row"):
                yield Label("Bank")
                yield Select(
                    [(bank, bank) for bank in self.bucket.banks],
                    id="bank",
                    allow_blank=False,
                    value=self.bank,
                )
                yield Label("Jog speed %")
                yield Input("50", id="speed", type="number")
            yield JogPad(id="jog")
            with Horizontal(classes="row"):
                yield Label("Position °")
                yield Input(placeholder="0-360", id="position", type="number")
                yield Button("Set position", id="set_position", variant="primary")
                yield Button("Stop", id="stop", variant="error")
                yield Button("Home", id="home")
                yield Button("Zero", id="zero", variant="warning")
            yield Static("", id="faults")
            yield Static("", id="tx")
        yield Footer()

    def on_mount(self) -> None:
        self.query_one("#monitor").border_title = f"{self.iface} · hi-can monitor"
        self.query_one("#commands").border_title = "bucket commands"
        self.query_one("#manual").border_title = "send manually"
        table = self.query_one(DataTable)
        for key, label, width in (
            ("id", "ID", 8),
            ("name", "Name", 38),  # system dropped: it's in the ID
            ("data", "Data", 23),
            ("value", "Value", 10),
            ("count", "Count", 6),
            ("hz", "Hz", 4),
        ):
            table.add_column(label, key=key, width=width)
        table.can_focus = False
        self.query_one(RichLog).can_focus = False
        self._set_filter(*self.filter)
        asyncio.get_running_loop().add_reader(
            self.bus.rx.fileno(), self._on_can_readable
        )
        self.timers = [
            self.set_interval(REFRESH_S, self._refresh),
            self.set_interval(JOG_PERIOD_S, self._jog_tick),
        ]
        self._show_jog()
        self.query_one(JogPad).focus()
        missing = [name for name in DECODERS if name not in self.hican.parameters]
        if missing:
            self.notify(
                "Decoders with no hi-can match: " + ", ".join(missing),
                severity="warning",
            )

    def on_unmount(self) -> None:
        asyncio.get_running_loop().remove_reader(self.bus.rx.fileno())

    # monitor

    def _on_can_readable(self) -> None:
        now = time.monotonic()
        _, address, mask = self.filter
        for can_id, extended, rtr, data in self.bus.receive():
            if extended and not rtr and can_id in self.fault_banks:
                self.faults[self.fault_banks[can_id]] = (data, now)
            if (
                self.rtr_request
                and extended
                and not rtr
                and can_id == self.rtr_request[0]
            ):
                self._show_rtr_reply(data, now)
            if (can_id & mask) != (address & mask):
                continue
            key = f"{can_id:08X}" if extended else f"{can_id:03X}"
            seen = self.seen.get(key)
            if seen is None:
                device, parameter, qualified = (
                    self.hican.describe(can_id)
                    if extended
                    else ("standard frame", "", None)
                )
                seen = self.seen[key] = Seen(device, parameter, DECODERS.get(qualified))
            seen.data, seen.rtr = data, rtr
            seen.count += 1
            seen.times.append(now)
            if self.show_log and not self.paused:
                self.pending_log.append(
                    f"{now - self.t0:10.3f}  {key:>8}  [{len(data)}]  {seen.data_text():<23}  "
                    f"{seen.device} {seen.parameter}  {seen.value_text()}"
                )

    def _refresh(self) -> None:
        now = time.monotonic()
        total = 0
        for seen in self.seen.values():
            while seen.times and now - seen.times[0] > 1.0:
                seen.times.popleft()
            total += len(seen.times)
        self.query_one("#rate", Label).update(
            f"{total} fr/s{'  PAUSED' if self.paused else ''}"
        )
        self._show_faults(now)
        if self.rtr_request and now - self.rtr_request[1] > RTR_REPLY_TIMEOUT_S:
            self.query_one("#manual_reply", Static).update(
                f"[yellow]no reply on {self.rtr_request[0]:08X} within {RTR_REPLY_TIMEOUT_S:.0f} s[/]"
            )
            self.rtr_request = None
        if self.paused:
            return
        if self.show_log:
            if self.pending_log:
                self.query_one(RichLog).write("\n".join(self.pending_log))
                self.pending_log.clear()
            return

        table = self.query_one(DataTable)
        added = False
        for key, seen in self.seen.items():
            cells = (
                seen.data_text(),
                seen.value_text(),
                str(seen.count),
                str(len(seen.times)),
            )
            if not seen.shown:
                name = f"{seen.device.split('.', 1)[-1]} {seen.parameter}"
                table.add_row(key, name, *cells, key=key)
                added = True
            else:
                for column, old, new in zip(
                    ("data", "value", "count", "hz"), seen.shown, cells
                ):
                    if old != new:
                        table.update_cell(key, column, new)
            seen.shown = cells
        if added:
            table.sort("id")

    def _set_filter(self, label: str, address: int, mask: int) -> None:
        self.filter = (label, address, mask)
        self.query_one(
            "#monitor"
        ).border_subtitle = f"{label}  {address:08X}/{mask:08X}"
        self.action_clear()

    def _apply_raw_filter(self, text: str) -> None:
        if not text:
            self._set_filter(
                *self.hican.filters[self.query_one("#filter", Select).value]
            )
            return
        address_text, _, mask_text = text.partition("/")
        try:
            address = int(address_text, 16)
            mask = int(mask_text, 16) if mask_text else self.hican.mask_all
        except ValueError:
            self.notify(f"Not a hex ID/MASK: {text}", severity="error")
            return
        self._set_filter("raw", address, mask)

    def action_toggle_view(self) -> None:
        self.show_log = not self.show_log
        self.query_one(DataTable).display = not self.show_log
        self.query_one(RichLog).display = self.show_log
        self.pending_log.clear()

    def action_toggle_pause(self) -> None:
        self.paused = not self.paused

    def action_clear(self) -> None:
        self.seen.clear()
        self.pending_log.clear()
        self.query_one(DataTable).clear()
        self.query_one(RichLog).clear()

    def action_focus_jog(self) -> None:
        self.query_one(JogPad).focus()

    async def action_quit(self) -> None:
        if self.jog_dir:
            self._send_speed(0)
        for timer in self.timers:
            timer.stop()  # else a last tick can land after the widgets are gone
        self.exit()

    # commands

    def on_select_changed(self, event: Select.Changed) -> None:
        if event.select.id == "manual_param":
            if isinstance(event.value, int):
                address = self.manual_params[event.value][1]
                self.query_one("#manual_id", Input).value = f"{address:08X}"
        elif event.select.id == "filter":
            self.query_one("#raw_filter", Input).value = ""
            self._set_filter(*self.hican.filters[event.value])
        elif event.select.id == "bank" and event.value != self.bank:
            if self.jog_dir:
                self.jog_dir = 0
                self._send_speed(0)
            self._disarm_zero()
            self.bank = event.value
            self._show_jog()

    def on_input_changed(self, event: Input.Changed) -> None:
        if event.input.id in ("manual_id", "manual_data"):
            self._update_manual_preview()

    def on_checkbox_changed(self, event: Checkbox.Changed) -> None:
        if event.checkbox.id == "manual_rtr":
            self.query_one("#manual_data", Input).placeholder = (
                "length to request, 0-8" if event.value else "hex bytes, e.g. 48 0D"
            )
            self._update_manual_preview()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id in ("manual_id", "manual_data"):
            self.manual_send()
        elif event.input.id == "raw_filter":
            self._apply_raw_filter(event.value.strip())
        elif event.input.id == "position":
            self.set_position()
        elif event.input.id == "speed":
            self.action_focus_jog()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        {
            "set_position": self.set_position,
            "stop": self.stop,
            "home": self.home,
            "zero": self.zero,
            "manual_send": self.manual_send,
        }[event.button.id]()

    def jog(self, direction: int) -> None:
        now = time.monotonic()
        repeat = direction == self.jog_dir and now < self.jog_until
        self.jog_dir = direction
        self.jog_until = now + (KEY_REPEAT_GAP_S if repeat else KEY_REPEAT_DELAY_S)
        if not repeat:
            self._send_speed(direction * self._jog_speed())
            self._show_jog()

    def _jog_tick(self) -> None:
        if not self.jog_dir:
            return
        if time.monotonic() < self.jog_until:
            self._send_speed(self.jog_dir * self._jog_speed())
        else:
            self.jog_dir = 0
            self._send_speed(0)
            self._show_jog()

    def step_bank(self, delta: int) -> None:
        banks = list(self.bucket.banks)
        self.query_one("#bank", Select).value = banks[
            (banks.index(self.bank) + delta) % len(banks)
        ]

    def stop(self) -> None:
        self.jog_dir = 0
        self._send_speed(STOP_NUDGE_PERCENT)
        self._send_speed(0)
        self._show_jog()

    def set_position(self) -> None:
        text = self.query_one("#position", Input).value
        try:
            degrees = float(text)
        except ValueError:
            self.notify("Position needs a number of degrees", severity="error")
            return
        if not 0 <= degrees <= 360:
            self.notify("Position must be 0-360°", severity="error")
            return
        self._send(
            "SET_POSITION",
            struct.pack("<h", round(degrees * POSITION_UNITS_PER_DEGREE)),
            f"{degrees:.1f}°",
        )

    def home(self) -> None:
        if HOME_PARAMETER not in self.bucket.parameters:
            self.notify(
                f"hi-can has no {BANK_PARAMETER}::{HOME_PARAMETER} yet",
                severity="warning",
            )
            return
        self._send(HOME_PARAMETER, b"", "")

    def zero(self) -> None:
        """First press arms, a second press within ZERO_CONFIRM_S zeroes both of the bank's encoders."""
        if ZERO_PARAMETER not in self.bucket.parameters:
            self.notify(
                f"hi-can has no {BANK_PARAMETER}::{ZERO_PARAMETER}", severity="warning"
            )
            return
        if self.zero_armed_bank == self.bank:
            self._disarm_zero()
            self._send(ZERO_PARAMETER, b"", "both encoders")
            return
        self.zero_armed_bank = self.bank
        self.query_one("#zero", Button).label = f"Zero {self.bank}?"
        self.zero_disarm_timer = self.set_timer(ZERO_CONFIRM_S, self._disarm_zero)

    def _disarm_zero(self) -> None:
        if self.zero_disarm_timer:
            self.zero_disarm_timer.stop()
            self.zero_disarm_timer = None
        self.zero_armed_bank = None
        self.query_one("#zero", Button).label = "Zero"

    def _show_faults(self, now: float) -> None:
        if not self.fault_banks:
            text = f"[dim]Fault  hi-can has no {FAULT_PARAMETER}[/]"
        else:
            parts = []
            for bank in self.bucket.banks:
                data, when = self.faults.get(bank, (None, 0.0))
                if data is None:
                    parts.append(f"{bank} [dim]no data[/]")
                    continue
                value = _fault(data) or f"bad length {len(data)}"
                colour = "green" if value == "ok" else "red"
                stale = " [dim](stale)[/]" if now - when > FAULT_STALE_S else ""
                parts.append(f"{bank} [{colour}]{value}[/]{stale}")
            text = "Fault  " + "  ·  ".join(parts)
        self.query_one("#faults", Static).update(text)

    # manual send

    def _manual_frame(self):
        """(address, data, rtr, dlc) from the manual panel, or an error message."""
        id_text = self.query_one("#manual_id", Input).value.strip()
        try:
            address = int(id_text, 16)
        except ValueError:
            return "ID must be hex"
        if not 0 <= address <= CAN_EFF_MASK:
            return "ID is over 29 bits"

        text = self.query_one("#manual_data", Input).value.strip()
        if self.query_one("#manual_rtr", Checkbox).value:
            try:
                dlc = int(text or "0")
            except ValueError:
                return "RTR: data is the length to request, 0-8"
            if not 0 <= dlc <= 8:
                return "RTR: length must be 0-8"
            return address, b"", True, dlc

        digits = "".join(ch for ch in text if ch not in " ,:-_")
        if len(digits) % 2:
            return "odd number of hex digits"
        try:
            data = bytes.fromhex(digits)
        except ValueError:
            return "data must be hex bytes"
        if len(data) > 8:
            return "more than 8 bytes"
        return address, data, False, len(data)

    def _update_manual_preview(self) -> None:
        preview = self.query_one("#manual_preview", Static)
        if not self.query_one("#manual_id", Input).value.strip():
            preview.update("")
            return
        frame = self._manual_frame()
        if isinstance(frame, str):
            preview.update(f"[red]{frame}[/]")
            return
        address, data, rtr, dlc = frame
        _, parameter, qualified = self.hican.describe(address)
        payload = f"R{dlc or ''}" if rtr else data.hex().upper()
        text = f"{address:08X}#{payload}  {parameter}"
        decoder = DECODERS.get(qualified)
        if decoder and not rtr and (value := decoder(data)):
            text += f" = {value}"
        preview.update(text)

    def manual_send(self) -> None:
        frame = self._manual_frame()
        if isinstance(frame, str):
            self.notify(frame, severity="error")
            return
        address, data, rtr, dlc = frame
        _, parameter, _ = self.hican.describe(address)
        if self._send_raw(address, data, rtr, dlc, f"manual {parameter}") and rtr:
            self.rtr_request = (address, time.monotonic())
            self.query_one("#manual_reply", Static).update(
                f"waiting for {address:08X}…"
            )

    def _show_rtr_reply(self, data: bytes, now: float) -> None:
        address, sent = self.rtr_request
        self.rtr_request = None
        _, parameter, qualified = self.hican.describe(address)
        decoder = DECODERS.get(qualified)
        value = decoder(data) if decoder else ""
        # Periodic broadcasts share the ID, so this is the first frame after the request.
        self.query_one("#manual_reply", Static).update(
            f"[green]reply[/] after {(now - sent) * 1000:.0f} ms: "
            f"{' '.join(f'{b:02X}' for b in data) or '(empty)'}  {parameter}"
            + (f" = {value}" if value else "")
        )

    def _jog_speed(self) -> float:
        try:
            return max(0.0, min(100.0, float(self.query_one("#speed", Input).value)))
        except ValueError:
            return 0.0

    def _show_jog(self) -> None:
        if self.jog_dir:
            state = f"{'▲' if self.jog_dir > 0 else '▼'} {self.jog_dir * self._jog_speed():+.0f} %"
        else:
            state = "idle"
        self.query_one(JogPad).update(
            f"[b]JOG {self.bank}[/b]  {state}    hold ↑/↓ to move · ←/→ bank · space stop · esc returns here"
        )

    def _send_speed(self, percent: float) -> None:
        duty = round(max(-100.0, min(100.0, percent)) * FULL_DUTY / 100)
        self._send("SET_SPEED", struct.pack("<h", duty), f"{percent:+.0f} %")

    def _send(self, parameter: str, data: bytes, description: str) -> None:
        address = self.bucket.address(self.bank, parameter)
        self._send_raw(
            address, data, False, 0, f"{self.bank} {parameter} {description}"
        )

    def _send_raw(
        self, address: int, data: bytes, rtr: bool, dlc: int, description: str
    ) -> bool:
        frame = f"{address:08X}#" + (f"R{dlc or ''}" if rtr else data.hex().upper())
        try:
            self.bus.send(address, data, rtr, dlc)
        except OSError as error:
            message = f"tx failed: {error.strerror or error}"
            if message != self.last_error:
                self.notify(message, severity="error")
            self.last_error = message
            self.query_one("#tx", Static).update(f"[red]{message}[/]  {frame}")
            return False
        self.last_error = ""
        self.query_one("#tx", Static).update(f"tx {frame}  {description}")
        return True


def main():
    parser = argparse.ArgumentParser(description="Bucket CAN console")
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--header", type=Path, default=HEADER)
    args = parser.parse_args()
    try:
        hican = HiCan(args.header)
        bucket = Bucket(hican)
    except KeyError as error:
        sys.exit(f"bucket_tui: {error.args[0]} not found in {args.header}")
    except ci.LibclangError as error:
        sys.exit(
            f"bucket_tui: {error}\nInstall the libclang wheel: pip install -r requirements.txt"
        )
    except (RuntimeError, OSError) as error:
        sys.exit(f"bucket_tui: {error}")
    try:
        bus = CanBus(args.iface)
    except OSError as error:
        sys.exit(f"bucket_tui: {args.iface}: {error.strerror or error}")
    BucketConsole(hican, bucket, bus, args.iface).run()


if __name__ == "__main__":
    main()
