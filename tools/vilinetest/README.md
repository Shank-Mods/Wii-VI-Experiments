# VI Line Test

Asks whether the Wii VI can show more than 1023 lines per field: the limit set by the
10-bit active-line count (ACV) in the VTR register, which rules out 1080p.

Two routes are tested: a hidden extra bit, and merging two progressive fields into one
frame so no single field needs more than 540 lines.

## Using it

- **Run all tests** (about 15 s; the screen blanks between steps). The results show on the
  menu and go to `vilinetest.txt` on the USB drive (or SD).
- **View: 480p built from two merged 240-line fields** shows the merge on a real display (see
  below). **View: … NOT merged** is the control. B, START, + or HOME leaves a view.

The disc-slot LED blinks slowly at the menu and fast while a test raster runs.

## Part 1: which register bits exist

With the VI stopped, each register is written with `FFFF` then `0000` and read back. Bits
that don't stick aren't implemented. The log flags any bits beyond the documented fields:
VTR bits 14-15 (above ACV), HLW bits 10-15, PRB/PSB bits 10-15 and so on.

## Part 2: what the extra bits do

Each test runs a 480p-rate raster at the stock clock and measures the line rate and the
length of each frame cycle from the beam counter (`DPV`, read as all 16 bits).

| Test | Setting | If wired | If ignored |
| --- | --- | --- | --- |
| B0 | 480p control | 1050 lines/cycle | |
| B1 | ACV 1000 | 2090 (counter wider than 11 bits) | 2048 (11-bit counter overflows) |
| B2 | ACV 1023 (documented max) | 2136 | 2048 |
| B3 | VTR bit 14 + ACV 100 | 2338 (bit 14 = ACV bit 10) | 290 |
| B4 | VTR bit 15 + ACV 100 | 4386 (bit 15 = ACV bit 11) | 290 |
| B5 | PRB bit 10 | 2074 | 1050 |
| B6 | PSB bit 10 | 2074 | 1050 |
| B7 | HLW bit 10 | 2906-clock line | 858-clock line |

## Part 3: merging two fields

In progressive mode the VI scans two "fields" per frame cycle. Each field has its own
vsync (EQU, shared by both fields) and blanking (separate odd and even registers).

The merge sets the blanking so field 1 has all of it before its picture and field 2 all
of it after. It then has the CPU follow the beam counter and set EQU to 0 during field 2,
restoring it before field 1. If the VI accepts that, the display sees one vsync per
frame and the two active areas back to back.

| Test | What | Merged | Not merged |
| --- | --- | --- | --- |
| C0 | progressive with different odd/even blanking | 1048 (fields alternate) | 1018 / 1078 (one set used) |
| C1 | EQU = 0 | 1050 (accepted) | |
| C2 | 480p from two 240-line fields | 525 | 534 |
| C2c | control, no EQU switch | | 534 |
| C3 | 1080p from two 540-line fields at 270 MHz | 1125 | 1134 |
| C3c | control | | 1134 |

### Seeing it

The C2 view is an ordinary 480p rate (31.5 kHz, 60 Hz), so any 480p display, GCVideo
included, can show it:

- **Merge works:** one stable picture, blue top half and red bottom half, a white diagonal
  running straight from corner to corner across the seam (yellow mark), and a tab on the
  right that's green on top and purple below.
- **Top half twice:** blue in both halves. The VI's second progressive field doesn't use
  the bottom framebuffer address. The timing can still be merged, but both halves would
  have to come from one buffer.
- **Rolling or no sync:** the second vsync is still there (compare with the NOT merged view,
  which should look like this).

C3 is the same thing at 1080p size and 58.7 Hz on a 270 MHz clock. It's measured only,
since no current display path can take it.
