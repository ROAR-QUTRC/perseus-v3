# Encoder Doom

The game runs on the encoder board's RP2350. `DOOM_PLATFORM` selects how it is
shown and controlled; only `web` exists so far. The engine is currently a test
pattern (`engine/test_pattern.cpp`) that proves the video and key path.
Architecture, the QOI decision and the HDMI dock pinout are in [docs/brief.md](docs/brief.md).

> Flashing `doom.uf2` replaces the encoder firmware on that board until the
> partition-table work lands. Reflash the encoder image afterwards.

## Web backend

| Item      | Value                                                                        |
| --------- | ---------------------------------------------------------------------------- |
| USB       | Composite: CDC serial (stdio) + CDC-NCM network adapter                      |
| Addresses | Board `192.168.7.1`; the PC gets `192.168.7.2` by DHCP (no gateway, no DNS)  |
| Page      | `http://192.168.7.1`, compiled into flash from `platform/web/index.html`      |
| Video     | `GET /stream`: `multipart/x-mixed-replace` of 320x200 palette PNGs in an `<img>` |
| Keys      | `POST /key`: `1<code>` down, `0<code>` up, `R` release all (`KeyboardEvent.code`) |
| Console   | One stats line per second while a viewer is connected                        |

| Core | Runs                                                                         |
| ---- | ---------------------------------------------------------------------------- |
| 0    | The game: `platform::display_present()` / `input_poll()`                     |
| 1    | TinyUSB, lwIP (no RTOS), HTTP server, PNG encoding                            |

- **Frame handoff.** One frame slot guarded by an atomic flag. A frame that
  arrives while core 1 is still busy is dropped, so the game never waits on the
  browser and latency stays at about one frame.
- **Browsers.** The PNG stream is verified in Firefox. Chrome is untested.

## Build and flash

```sh
cmake -S . -B build -G Ninja      # -DDOOM_PLATFORM=web is the default
cmake --build build
```

Hold BOOTSEL, plug in, copy `build/doom.uf2` to the drive. Then open
`http://192.168.7.1` once NetworkManager has brought the link up.

## Layout

| Path                         | Contents                                              |
| ---------------------------- | ----------------------------------------------------- |
| `platform.hpp`               | Display and input interface the engine uses           |
| `engine/`                    | The game (test pattern for now)                       |
| `platform/web/`              | USB network, HTTP, PNG, key map, page                 |
| `platform/hdmi/`             | TBD: HSTX DVI on the USB-C pairs + USB-host keyboard  |
