# Encoder Doom: Brief

Doom running on the encoder board's RP2350, shown and controlled through a
swappable backend. Branch `feat/encoder-of-doom`. Status (2026-09-30): the web
backend is built and PC-tested with a test pattern standing in for the engine;
it has not yet run on hardware.

## 1. Architecture

```
core 0: engine                      platform.hpp                  core 1: backend
  320x200 palette frame + palette  ─────────────────►  web:  USB NCM → lwIP → HTTP → browser
  HID key events                   ◄─────────────────  hdmi: HSTX DVI + USB-host keyboard (TBD)
```

| Piece          | Design                                                                                   |
| -------------- | ---------------------------------------------------------------------------------------- |
| Interface      | `display_set_palette()`, `display_present()`, `input_poll()`; keys are USB HID usage IDs  |
| Backend choice | Build time: `-DDOOM_PLATFORM=web` (default) or `hdmi` (TBD)                               |
| Frame handoff  | One slot, atomic flag; a frame arriving while core 1 is busy is dropped, never queued     |
| Key handoff    | `pico_util` queue, core 1 → core 0                                                        |
| Boot (planned) | Doom in its own flash partition; a bench-only trigger reboots into it, a power cycle returns to the encoder. Today `doom.uf2` replaces the encoder firmware |

### Web backend

| Item    | Value                                                                                         |
| ------- | --------------------------------------------------------------------------------------------- |
| USB     | Composite: CDC serial (stdio) + CDC-NCM network adapter                                       |
| Network | Board `192.168.7.1`, PC `192.168.7.2` by DHCP; no gateway or DNS advertised                    |
| Stack   | TinyUSB + lwIP without an RTOS, polled from core 1's loop; own HTTP server on the raw TCP API   |
| Page    | `GET /`: `index.html`, compiled into flash; about 6 lines of JavaScript, keys only             |
| Video   | `GET /stream`: `multipart/x-mixed-replace` of palette PNGs, displayed by a plain `<img>`       |
| Encoder | Fixed-Huffman deflate; candidates from a 3-byte hash, the previous byte and the row above      |
| Keys    | `POST /key`: `1<code>` down, `0<code>` up, `R` release all; sent in order, repeats ignored     |
| RAM     | 325 KB of 520 KB, including the test pattern's 64 KB frame that Doom's screen buffer replaces  |

Verified on a PC: PNGs round-trip (CRC, Adler-32, pixels); Firefox plays the PNG
stream live; the page's key POSTs arrive in order. Untested: hardware, Chrome.

## 2. QOI

Considered and rejected for this stream. QOI encodes RGB, so a palette frame
starts at 3 bytes per pixel against PNG's 1, and its 64-colour cache is smaller
than Doom's 256-colour palette. Browsers also can't decode it natively, so it
would need a JavaScript decoder and a canvas.

| Frame (320x200) | Raw 8-bit | Our PNG | QOI     |
| --------------- | --------- | ------- | ------- |
| Blank           | 64,000    | 1,251   | 1,056   |
| Textured        | 64,000    | 13,470  | 62,610  |
| Noise           | 64,000    | 68,554  | 208,148 |

Fallback if PNG encoding turns out to be the bottleneck on hardware: a QOI-style
scheme over palette indices (no match search), at the cost of a JavaScript decoder.

## 3. Flash partitioning (plan)

Goal: the encoder and Doom share a board, and nothing about Doom can affect the
encoder's normal operation. The RP2350 boot ROM reads a partition table, starts
the first valid image, and maps whichever partition it starts so that it appears
at `0x10000000`. Both builds therefore stay ordinary, unrelocated builds.

| Region          | Offset     | Size   | Contents                                                        |
| --------------- | ---------- | ------ | --------------------------------------------------------------- |
| Partition table | `0x000000` | 4 KB   | Created with `picotool partition create layout.json`             |
| P0 `encoder`    | `0x001000` | 1 MB   | Encoder firmware; first partition, so the default boot           |
| P1 `doom`       | `0x101000` | ~8 MB  | Doom engine, web backend and WAD bundled in one image            |
| P2 `zero`       | `0xFFE000` | 4 KB   | Zero-offset record, same place as today, now reserved by the table |
| Unpartitioned   | `0xFFF000` | 4 KB   | Left free for the RP2350-E10 absolute UF2 block                  |

**Switching (boot ROM only, no launcher).** The encoder's trigger looks up
partition `doom` by name and calls the boot ROM's reboot with the flash-update
boot type pointing at it, so the next boot prefers Doom once. Doom exits with a
normal reboot; reset and power cycles always start the encoder. Still to verify
in the RP2350 datasheet: this boot type means "boot this partition once" for two
unrelated images, not only for A/B pairs.

**Protecting the encoder**

| Rule                        | How                                                                                          |
| --------------------------- | -------------------------------------------------------------------------------------------- |
| Encoder always boots        | P0 is the default; no "boot Doom" state is ever stored, so every reset returns to the encoder |
| Rover can't start Doom      | Trigger only on the USB console, never Modbus/CAN; compiled in only by a bench build option    |
| Missing Doom is harmless    | Trigger checks the `doom` partition first; if it's absent or invalid, it logs and carries on   |
| Zero record untouched       | Stays at `0xFFE000` in its own partition; Doom never links flash writes                        |
| Zero record still readable  | `zero_store` reads through `XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE`, not `XIP_BASE`, because translation remaps `XIP_BASE` to the running partition. Works with or without a table |
| Encoder workflow unchanged  | P0 accepts the standard `rp2350-arm-s` UF2 family; P1 takes a custom family (or `picotool load -p 1`) |
| Easy rollback               | `picotool erase`, then flash `encoder.uf2`: back to today's single-image layout                |

**Order of work:** the encoder changes first (untranslated zero read, trigger
behind a build option), tested with no partition table. Then the table and
Doom in P1. Then on the bench: power cycle, reset, a missing Doom image,
trigger → Doom → reset, with the zero surviving all of it.

## 4. HDMI dock (future)

The USB-C port's four high-speed pairs are wired to the RP2350's HSTX pins, so it
can carry DVI (640x480 at 60 Hz, Doom's frame doubled). The dock is a custom
passive adapter: USB-C plug → HDMI socket + USB-A socket (keyboard) + 5 V input.

| Lane      | RP2350      | USB-C     | HDMI    |
| --------- | ----------- | --------- | ------- |
| D2+ / D2− | GPIO12 / 13 | A2 / A3   | 1 / 3   |
| CK+ / CK− | GPIO14 / 15 | B10 / B11 | 10 / 12 |
| D1+ / D1− | GPIO16 / 17 | B2 / B3   | 4 / 6   |
| D0+ / D0− | GPIO18 / 19 | A11 / A10 | 7 / 9   |

| Other         | Wiring                                                                  |
| ------------- | ----------------------------------------------------------------------- |
| Ground        | USB-C GND → HDMI shields 2, 5, 8, 11 and pin 17                          |
| 5 V           | Dock supply → USB-C VBUS (powers the board), HDMI pin 18, USB-A VBUS     |
| Keyboard      | USB-C D+/D− → USB-A D+/D−; the RP2350 runs as USB host                   |
| Not connected | HDMI 13 (CEC), 14, 15/16 (DDC), 19 (hot plug); USB-C SBU is unconnected  |

Wire net-for-net as in the table. The CK pair lands on the connector's RX1 pins
with swapped polarity, which HSTX can invert in firmware anyway.

### Caveats

- **Not a standard adapter.** Off-the-shelf USB-C→HDMI adapters expect DisplayPort
  Alt Mode and won't work with raw TMDS.
- **Never drive HSTX into a PC.** Those pins are a PC's USB 3 lanes, and its receivers
  are wired to the connector. Enable HSTX only after detecting a monitor: read
  GPIO12–19 with pull-downs; a monitor's 50 Ω terminations to 3.3 V pull all eight high.
- **No EDID or hot-plug.** Without DDC or pin 19, the resolution is fixed at
  640x480 at 60 Hz, the mode every display must accept.
- **One plug orientation.** A flipped plug swaps D2↔D1 and D0↔CK. HSTX can remap
  lanes in firmware; otherwise mark the plug.
- **No ESD or series resistors on the HSTX pairs** of this PCB. Compare against a
  known RP2350 HSTX DVI design before connecting a monitor.
- **The board can't power the monitor's 5 V.** VBUS is input-only (through D5), so
  the dock supplies it.
- **DVI, not HDMI.** No audio. Keep the dock's traces and cable short and the pairs
  length-matched.
