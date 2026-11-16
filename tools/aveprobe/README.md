# aveprobe

Reads and writes the AV encoder's (AVE-RVL) I2C registers one step at a time, with a test
pattern and a machine-readable step code on screen, so a capture can tie each picture to the
register state that produced it. Runs from the Homebrew Channel (`<ahb_access/>`) or the Wii
test bench (`recipes/aveprobe.json` in PrivateTestWii). Everything it writes is volatile: the
AVE is reprogrammed by every `VIDEO_Init` and reset by a power cycle.

## Arguments

`sd:/apps/aveprobe/wtb.txt`, one per line (`cfg name value`), or `cfg=name=value` from the bench:

| Name | Value | Meaning |
| --- | --- | --- |
| `dump` | `1` | try reading registers 00..FF back (write the register, repeated start, 0xE1) and log what answers |
| `hold` | ms | time each step stays up (default 3000) |
| `w` | `RR:VV[+RR:VV]` | one step: write hex bytes `VV` (2 digits per byte: `71:8e8e` is 2 bytes) to register `RR`; `+` joins writes in one step. Repeatable, run in order |
| `sweep` | `RR:A-B` | one step per value A..B (hex) written to register RR |
| `keep` | `1` | don't restore libogc's encoder settings between steps |

Step 0 is the baseline (libogc's settings, re-written). After the last step the baseline is
restored and the tool exits.

## Screen

8 colour bars, a 16-step grey ramp, a 1-pixel vertical line grating (left) and a horizontal
line grating (right), and along the bottom the step code: a white guard block, 12 bits (MSB
first, white = 1) on a grey band, a white guard block.

## Log

`ave.txt` (sent to the bench at the end, each line also as a LOG line): the video mode, the
read-back dump, and one line per step with the time it went up (ms since start) and each
write's ACK count (`got/expected`: address + register + data bytes).

## Known registers

See [wiibrew: Hardware/AV Encoder](https://wiibrew.org/wiki/Hardware/AV_Encoder). Candidates
for other timings: 0x00 bit 0 ("D-Terminal"?), 0x01 bits 0-1 (3 = "DEBUG"?), 0x65 (DAC
control / oversampling, libogc writes 3), 0x0A, 0x6A, and the unknown 0x63, 0x64, 0x68, 0x6B,
0x6C, 0x70. 0x67 bit 0 shows the AVE's own test pattern.
