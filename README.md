# consolized-game-boy

This repository contains the current state of the consolized Game Boy project, based on Andy West's original Game Guy project. It's evolved a bit since then, now supporting multiple controller options, custom PCBs, RP2040 and RP2350 support and a newer HDMI-compatible DVI output path with audio (no VGA2HDMI adapter required).

The core idea is still the same: build a self-contained Game Boy console around harvested Game Boy hardware, with video, controls, power, and enclosure work integrated into a compact custom build.

Recent major addition:
- 2026-04-04: HDMI-compatible DVI firmware and hardware path, including digital video plus audio output.

![gameplay](./images/gameplay.gif?raw=true)

![front closed](./images/front_closed.jpg?raw=true)

![front open](./images/front_open.jpg?raw=true)

![back](./images/back.jpg?raw=true)

![hdmi osd](./images/HDMI_osd.jpg?raw=true)

![screencap 1](./images/screencap1.jpg?raw=true)

![screencap 2](./images/screencap2.jpg?raw=true)

HDMI (DVI)

![hdmi clean](./images/hdmi_clean.jpg?raw=true)

## What This Project Includes

- Single-Pico firmware designs
- VGA output firmware with RGB222 and RGB332 variants
- HDMI-compatible DVI output firmware with audio
- RP2040 and RP2350 build targets
- NES shift-register controller support
- NES Classic / Wii I2C controller support
- On-screen display for settings and runtime controls
- Saved settings in flash
- Custom motherboard and breakout PCBs
- 3D-printable case and mount parts
- Support for harvesting CPU and RAM from a Super Game Boy SNES/Famicom cartridge
- Link port support, with credit to Markeno for the PCB mod

## Repository Layout

| Path | Purpose |
| --- | --- |
| `src/gb_vga` | VGA firmware project |
| `src/gb_dvi` | HDMI-compatible DVI firmware project |
| `src/buildall.cmd` | Batch wrapper that builds every currently scripted firmware variant |
| `src/find_picotool.cmd` | Shared helper that tries to reuse an existing `picotool` build |
| `src/find_rpi_rp2_drive.cmd` | Shared helper that detects a mounted `RPI-RP2` device |
| `docs` | BOMs and PDF schematics |
| `easyeda` | EasyEDA project archives |
| `gerber` | Gerber archives for motherboard and breakout boards |
| `3D` | STL files for mounts, controller jack holders, and case-specific pieces |
| `images` | Photos, screenshots, and wiring / PCB reference images |

## Hardware Assets

The repository now contains the hardware files needed to reproduce both the VGA and DVI versions.

### Documentation

- `docs/VGA_BOM_V3_2_2_SGB_CART_2023-01-14.csv`
- `docs/VGA_schematic_sgb_cart_V3.2.2_2023-01-18.pdf`
- `docs/HDMI_BOM_SGB_CART.csv`
- `docs/HDMI_schematic_dvi_V2.pdf`

### PCB Archives

EasyEDA source archives are in `easyeda/`.

Gerbers are in `gerber/`, including:

- VGA SGB-cart motherboard variants
- VGA SMD-cart motherboard variant
- HDMI / DVI SGB-cart motherboard variant
- Link port breakout
- NES breakout
- Power / reset board
- USB with data board
- Wii jack breakout

### 3D Printed Parts

STLs are in `3D/`, including VGA and HDMI-specific mounts and Wii jack holder parts.

## Firmware Variants

There are now two firmware families in the repo.

### 1. VGA firmware

Project path:

- `src/gb_vga`

Prebuilt release naming:

- `gb_vga_<platform>_<video>_<controller>.uf2`

Examples:

- `gb_vga_rp2040_rgb222_nes.uf2`
- `gb_vga_rp2040_rgb332_wii.uf2`
- `gb_vga_rp2350_rgb222_nes.uf2`

Options:

- `platform`: `rp2040` or `rp2350`
- `video`: `rgb222` or `rgb332`
- `controller`: `nes` or `wii`

Notes:

- `rgb222` uses fewer GPIOs and is the default VGA output mode in the build script.
- `wii` means the NES Classic / Wii I2C controller path.
- `nes` means the original shift-register controller path.

### 2. HDMI-compatible DVI firmware

Project path:

- `src/gb_dvi`

Prebuilt release naming:

- `dmg_dvi_<platform>_res<mode>_<controller>.uf2`

Examples:

- `dmg_dvi_rp2040_res0_nes.uf2`
- `dmg_dvi_rp2040_res2_wii.uf2`
- `dmg_dvi_rp2350_res1_wii.uf2`

Options:

- `platform`: `rp2040` or `rp2350`
- `mode`: `0`, `1`, or `2`
- `controller`: `nes` or `wii`

Display modes:

- `0`: 640x480 output, horizontally scaled x4 and vertically x3 for full-screen 640x480 presentation
- `1`: 800x600 output, horizontally scaled x4 and vertically x4 for a 640x576 window
- `2`: 640x480 output, horizontally scaled x2 and vertically x2 for a 320x288 window

## Prebuilt Firmware

If you do not want to build from source, use the `.uf2` files from the [Releases](https://github.com/joeostrander/consolized-game-boy/releases) page.

Current release coverage in this repository includes:

- 12 DVI variants: 2 platforms x 3 display modes x 2 controller types
- 8 VGA variants: 2 platforms x 2 video formats x 2 controller types

Choose the file that matches:

1. Your board: `rp2040` for Pico, `rp2350` for Pico 2
2. Your output path: `gb_vga_*` for VGA, `dmg_dvi_*` for HDMI-compatible DVI
3. Your controller wiring: `nes` or `wii`
4. Your display mode: `rgb222` / `rgb332` for VGA, or `res0` / `res1` / `res2` for DVI

To flash, hold BOOTSEL on the Pico / Pico 2, connect USB, then copy the matching `.uf2` to the `RPI-RP2` drive.

## Building From Source

The firmware projects can now be built from this repository directly; they do not need to live under `C:\VSARM\sdk\pico`.

### Common Requirements

- Windows build environment with `cmake` and MinGW available on `PATH`
- A valid Pico SDK checkout exposed via `PICO_SDK_PATH`
- `picotool` available if you want `.uf2` output

The wrapper scripts try to reuse an existing `picotool` package from a sibling VSARM `gb_vga` or `gb_dvi` build when one is available.

If a target device is mounted as `RPI-RP2`, the build scripts will also try to copy the generated `.uf2` automatically.

### Building VGA

Extra requirement for VGA:

- Pico Extras must be available through `PICO_EXTRAS_PATH`, or placed as a sibling of the SDK checkout so `pico_scanvideo_dpi` can be found.

Build from `src/gb_vga`:

```bat
build_clean.cmd
build_clean.cmd rp2040 rgb222 nes
build_clean.cmd rp2040 rgb332 wii
build_clean.cmd rp2350 rgb222 nes
build_clean.cmd rp2350 rgb332 wii
```

Argument format:

```text
build_clean.cmd [rp2040|rp2350] [rgb222|rgb332] [nes|wii]
```

### Building DVI

Build from `src/gb_dvi`:

```bat
build_clean.cmd
build_clean.cmd rp2040 0 nes
build_clean.cmd rp2350 2 wii

build.cmd
build.cmd rp2040 0 nes
build.cmd rp2350 2 wii
```

Argument format:

```text
build_clean.cmd [rp2040|rp2350] [0|1|2] [nes|wii]
build.cmd       [rp2040|rp2350] [0|1|2] [nes|wii]
```

Notes:

- `build_clean.cmd` always recreates the build directory.
- `build.cmd` tries to reuse the existing build and will recreate it if cached settings no longer match the requested platform, resolution mode, or controller type.

### Building Every Scripted Variant

From `src/`:

```bat
buildall.cmd
```

This runs the full scripted matrix for both projects:

- 12 DVI builds
- 8 VGA builds

It stops on the first failure so the failing variant is easy to identify.

## Project Notes

- VGA uses Pico Extras because `pico_scanvideo_dpi` is provided there.
- DVI does not need Pico Extras.
- The DVI firmware supports audio output in addition to digital video.
- Both firmware families expose controller selection at build time.
- Both firmware families now support RP2040 and RP2350 targets.

## Videos

Demo:

[![RGB332 Video Demo](https://img.youtube.com/vi/dT88XEHc5w8/0.jpg)](https://youtu.be/dT88XEHc5w8)

Andy liked my modifications and made an updated video where he built one with my changes:

[![The Game Guy Mini, Upgrading the Unportable Game Boy!](https://i.ytimg.com/vi/gPNHySf-hk0/0.jpg)](https://youtu.be/gPNHySf-hk0)

Original video:

[![Game Guy - The Unportable Game Boy](https://img.youtube.com/vi/ypGMU5lLjeU/0.jpg)](https://www.youtube.com/watch?v=ypGMU5lLjeU)

element14 Community pages:

- https://community.element14.com/challenges-projects/element14-presents/project-videos/w/documents/27407
- https://community.element14.com/w/documents/27862/episode-577-the-game-guy-mini-upgrading-the-unportable-game-boy

## Credits

- Andy West for the original concept and code
- Joe Ostrander for the later firmware, hardware, controller, OSD, flash-save, scaling, and newer DVI work in this repo
- Markeno for the link port PCB mod

## Contact
For questions or collaboration, reach me at:  
joeostrander [at] protonmail [dot] com  

