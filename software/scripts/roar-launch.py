#!/usr/bin/env python3
"""Open this machine's zellij launch session from config/launch_profiles.toml.

Looks up the device (the hostname, or --device), writes a zellij layout for its tabs and
panes, and starts zellij inside `devenv --profile <profile> shell` so every pane has the
ROS and CycloneDDS environment. Pane commands start suspended: they wait for ENTER.

If the session already exists it is attached instead; --fresh kills it and starts over.
"""

import argparse
import os
import shlex
import socket
import subprocess
import sys
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    # Python < 3.11 (Ubuntu 22.04 on the Jetsons) has no tomllib, so borrow a newer
    # interpreter from nix rather than depend on a pip package.
    if os.environ.get("ROAR_LAUNCH_REEXEC"):
        sys.exit("roar-launch: needs Python 3.11+ for tomllib")
    os.environ["ROAR_LAUNCH_REEXEC"] = "1"
    os.execvp(
        "nix",
        [
            "nix",
            "shell",
            "nixpkgs#python3",
            "--command",
            "python3",
            __file__,
            *sys.argv[1:],
        ],
    )

CONFIG = "config/launch_profiles.toml"
DEFAULT_PROFILE = "cyclonedds"
DEFAULT_SOURCE = "software/ros_ws/install/setup.bash"


def repo_root() -> Path:
    if root := os.environ.get("DEVENV_ROOT"):
        return Path(root)
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            capture_output=True,
            text=True,
            check=True,
        )
        return Path(out.stdout.strip())
    except (subprocess.CalledProcessError, FileNotFoundError):
        sys.exit("roar-launch: run this from inside the perseus-v3 repo")


def kdl(value: str) -> str:
    """Quote a string for KDL."""
    escaped = value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    return f'"{escaped}"'


# Per-device keys that are settings; every other key is a tab.
SETTINGS = ("session", "devenv_profile", "source")


def pane_kdl(pane: str | dict, indent: str) -> str:
    if isinstance(pane, str):
        pane = {"cmd": pane}
    command = pane.get("cmd", "")
    name = pane.get("name", command)
    name = f" name={kdl(name)}" if name else ""
    if not command:
        return f"{indent}pane{name}"
    lines = [
        f'{indent}pane{name} command="bash" {{',
        f'{indent}    args "-c" {kdl(command)}',
    ]
    if not pane.get("autostart", False):
        lines.append(f"{indent}    start_suspended true")
    lines.append(f"{indent}}}")
    return "\n".join(lines)


def layout_kdl(tabs: dict, cwd: Path) -> str:
    out = [
        "layout {",
        f"    cwd {kdl(str(cwd))}",
        "    default_tab_template {",
        '        pane size=1 borderless=true { plugin location="zellij:tab-bar"; }',
        "        children",
        '        pane size=2 borderless=true { plugin location="zellij:status-bar"; }',
        "    }",
    ]
    for i, (name, panes) in enumerate(tabs.items()):
        if not isinstance(panes, list):
            panes = [panes]
        focus = " focus=true" if i == 0 else ""
        out.append(f"    tab name={kdl(name)}{focus} {{")
        out.append('        pane split_direction="vertical" {')
        out.extend(pane_kdl(p, "            ") for p in panes or [""])
        out.append("        }")
        out.append("    }")
    out.append("}")
    return "\n".join(out) + "\n"


def session_state(name: str) -> str | None:
    """Return "running", "exited" or None for a zellij session."""
    out = subprocess.run(
        ["zellij", "list-sessions", "--no-formatting"],
        capture_output=True,
        text=True,
    )
    for line in out.stdout.splitlines():
        if line.split(" ", 1)[0] == name:
            return "exited" if "EXITED" in line else "running"
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--device", help="device entry to use instead of this hostname")
    parser.add_argument("--config", type=Path, help=f"default: <repo>/{CONFIG}")
    parser.add_argument(
        "--dry-run", action="store_true", help="print the layout and exit"
    )
    parser.add_argument(
        "--fresh", action="store_true", help="kill an existing session first"
    )
    parser.add_argument("--list", action="store_true", help="list configured devices")
    args = parser.parse_args()

    root = repo_root()
    config_path = args.config or root / CONFIG
    with open(config_path, "rb") as f:
        config = tomllib.load(f)

    if args.list:
        print("\n".join(config))
        return

    device = args.device or socket.gethostname()
    if device not in config:
        sys.exit(
            f"roar-launch: no entry for '{device}' in {config_path}\n"
            f"  configured: {', '.join(config) or 'none'} (pick one with --device)"
        )
    entry = config[device]
    tabs = {k: v for k, v in entry.items() if k not in SETTINGS}
    if not tabs:
        sys.exit(f"roar-launch: '{device}' has no tabs in {config_path}")

    session = entry.get("session", device)
    profile = entry.get("devenv_profile", DEFAULT_PROFILE)
    source = root / entry.get("source", DEFAULT_SOURCE)
    layout = layout_kdl(tabs, root)

    if args.dry_run:
        print(layout, end="")
        return

    if os.environ.get("ZELLIJ"):
        sys.exit("roar-launch: already inside zellij; detach first (Ctrl-o d)")

    # An exited session would be resurrected by a server started outside devenv, with
    # none of its environment, so only a running one is worth attaching to.
    state = session_state(session)
    if state:
        if state == "running" and not args.fresh:
            print(
                f"roar-launch: attaching to existing session '{session}' (--fresh to restart)"
            )
            os.execvp("zellij", ["zellij", "attach", session])
        subprocess.run(["zellij", "delete-session", "--force", session], check=False)

    runtime = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp"))
    layout_path = runtime / f"roar-launch-{session}.kdl"
    layout_path.write_text(layout)

    # Sourced here rather than per pane, so plain shells get the workspace too.
    inner = f"exec zellij --session {shlex.quote(session)} --new-session-with-layout {shlex.quote(str(layout_path))}"
    if source.is_file():
        inner = f"source {shlex.quote(str(source))} && {inner}"
    else:
        print(
            f"roar-launch: {source} not found, starting without it (colcon build first?)"
        )

    print(
        f"roar-launch: '{device}' -> session '{session}' in devenv profile '{profile}'"
    )
    os.chdir(root)
    os.execvp(
        "devenv", ["devenv", "--profile", profile, "shell", "--", "bash", "-c", inner]
    )


if __name__ == "__main__":
    main()
