# Wii-VI-Experiments

Reprogramming the Nintendo Wii (Hollywood) Video Interface **pixel clock** above its stock
54 MHz, toward HD-capable pixel rates (74.25 MHz for 720p/1080i, 148.5 MHz), through
clean-room reverse engineering of the undocumented VI PLL registers.

## Status

- ✅ **VI clock PLL fully decoded and confirmed on hardware.** The pixel clock follows an
  exact, linear law and is reprogrammable from 54 MHz to at least 193 MHz with no lock loss.
- ✅ **Exact HD pixel clocks reached:** 74.25 MHz (measured exact) and 148.5 MHz (±0.01%).
- ✅ **720p raster locked by a real monitor** at 74.25 MHz (44.94 kHz line, ~58.5 Hz).
- 🚧 **720p colour** comes out a green/magenta mess — chroma phase vs GCVideo (>54 MHz)
  undersampling is being isolated now.

## The result in one line

> **F = 54 MHz × Nfb / V**, where **Nfb = `((0x1C4>>8)&0xFF)+1`** and **V = `(0x1C8>>20)&0xF`**
> (both 7 at stock → 54 MHz). Full register documentation and the measured data:
> **[docs/VI_PLL.md](docs/VI_PLL.md)**.

| Target | Nfb | V | `HW_0x1C4` | `HW_0x1C8` | measured |
| --- | ---: | ---: | --- | --- | --- |
| 54 MHz (stock) | 7 | 7 | `0x002006C0` | `0xD870700E` | 54 000 kHz |
| 74.25 MHz (720p) | 11 | 8 | `0x00200AC0` | `0xD880700E` | 74 250 kHz |
| 148.5 MHz | 22 | 8 | `0x002015C0` | `0xD880700E` | 148 512 kHz |

## Tools (`tools/`)

Each is a Homebrew Channel app (`<ahb_access/>` for Hollywood register access). They measure
the real VI clock against the CPU timebase — no working display required — and log to the
USB drive. All touched registers reset on power cycle.

| Tool | What it does |
| --- | --- |
| **vi_probe** | Read-only. Logs VI timing + Hollywood clock/PLL registers in stock modes. Writes nothing. |
| **vi_pll** | Writes the PLL candidates, measures the resulting clock, logs before/after with write-ahead logging and auto-restore. Default sweep confirms the HD targets; a `vi_pll.cfg` on the USB root overrides it. |
| **vi720test** | Sets 74.25 MHz, programs a 720p-rate raster, draws a test pattern, and measures the real line/field/total-lines. Colour-diagnostic patterns (grayscale / primaries / fine). |

## Build

Requires devkitPPC + libogc + libfat (and libwiiuse/libbte for `vi_pll`/`vi720test`). Per tool:

```sh
cd tools/<tool> && make
```

Install `boot.dol` + `meta.xml` to `apps/<tool>/` on an SD/USB drive and launch from the
Homebrew Channel. CI (`.github/workflows/build.yml`) builds all three in the official
`devkitpro/devkitppc` container and uploads the `boot.dol`s as artifacts.

## Data (`logs/`)

Raw on-hardware logs that back every number in the docs: the `vi_pll` field-map, full sweep,
and HD-target runs, and the `vi720` 720p-lock run.

## Safety

Low-risk by design, but note: **never write `0x1C4` byte 2 to anything but `0x20`** — `0x21`
hard-hangs the console (recoverable by power cycle). Everything resets on reboot; the tools
restore originals after each experiment and log write-ahead so a hang is always diagnosable.

## Credit

VI reverse engineering and tooling by Shank-Mods. Builds on the 1280×240p VI work and the
read-only `vi_probe`. See `docs/original-briefing.docx` for the project background.
