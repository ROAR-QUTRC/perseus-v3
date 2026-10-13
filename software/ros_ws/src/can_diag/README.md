# can_diag

Generic hi-can diagnostic CLI. Sends an arbitrary hi-can packet (an RTR
request or a plain data frame) to any address and prints/decodes whatever
response arrives. Not tied to any one subsystem or packet type — it's meant
to be reused for debugging any parameter group as firmware work continues.

## Usage

List known preset addresses:

```sh
ros2 run can_diag can_diag --list-presets
```

Send a GET_ANGLE RTR request to the left lift encoder and print the decoded
angle:

```sh
ros2 run can_diag can_diag --iface can0 --preset excavation.bucket.encoder.lift_l.get_angle --rtr
```

(`--rtr` is technically redundant here since the preset defaults to RTR, but
spelling it out is harmless and makes the intent explicit.)

Listen for the left lift motor bank's periodic current broadcast (it's never
sent in response to an RTR request, so `--listen` is required instead of
`--rtr`):

```sh
ros2 run can_diag can_diag --iface can0 --preset excavation.bucket.motor_bank.lift.get_current --listen --timeout 1000
```

Send an arbitrary raw address with a custom payload, no preset:

```sh
ros2 run can_diag can_diag --iface vcan0 --system 0x04 --subsystem 0x00 --device 0x00 --group 0x00 --parameter 0x02 --data 00,64
```

Run `ros2 run can_diag can_diag --help` for the full flag list.

## Adding a new preset/decoder

There's no runtime registry of parameter groups in `hi_can` — this tool
keeps one small hand-maintained table instead
(`include/can_diag/known_packets.hpp`). To add a new decodable packet type:

1. Locate (or add) the `standard_address_t`/enum combination for it in
   `hi_can_address.hpp`'s addressing namespace.
2. Locate (or add) the corresponding payload typedef in
   `hi_can_parameter.hpp` (`SimpleSerializable<T>`/`scaled_int16_t<N>`/etc —
   these are all constructible from a `std::vector<uint8_t>`).
3. Push a new `known_packet_t` entry in `known_packets()` with a decode
   lambda that constructs that type from the data vector and formats its
   value(s), following the existing encoder/motor-bank entries as a
   template.
4. Rebuild (`colcon build --packages-up-to can_diag`).

Anything not registered still works — the tool just falls back to a raw hex
dump of the response instead of a decoded value (or pass `--raw` to force
that even for a registered address).

## Note on hi_can_raw resolution

Like `can_if` and the native examples, this package's `CMakeLists.txt` does
`find_package(hi_can_raw QUIET)` first and only falls back to building
`software/shared/hi-can-raw` from source if no package is found — this lets
CI/CD's Nix-packaged, cached build of `hi_can_raw` get reused instead of
rebuilding it every time. One consequence: if you edit
`software/shared/hi-can`/`hi-can-raw` locally without going through a Nix
rebuild, `can_diag` (and every other package here) will keep linking
against whatever `hi_can_raw` package is already in your environment until
that's refreshed.
