# Mono Ringmod

The RINGMOD machine of the Elektron Monomachine (FX-RINGMOD) as a CLAP
effect for Bitwig (macOS, universal binary).

The DSP is a fixed-point C port of the machine in Monomachine OS 1.32B,
written from its DSP56300 code and checked sample for sample against that
code running in an emulator. No firmware code is in this repository. The
machine's tables are generated from formulas at start-up, except one: the
oscillator's band-limited impulse (515 words), which no formula reproduces
and which is copied from the firmware.

## Controls

| Parameter     | Range | On the Monomachine | What it does |
|---------------|-------|--------------------|--------------|
| Pitch         | 0–127 | the track's note | The carrier's pitch as a MIDI note: 60 is 261.5 Hz, 69 is 439.8 Hz. The display shows the frequency. |
| Note Tracking | Off / On | (none) | On (default): notes into the plug-in's note input move the carrier, a note n by n − 60 semitones from Pitch. Off: Pitch alone sets it. |
| Wave          | 0–127 | WAVE | The carrier's shape: a sine at 0, a rounded square at 64, a square with a droop at 127, crossfading in between. The shapes other than the sine are quieter (about −9.5 dB at 127). |
| Ext           | 0–127 | EXT  | Crossfades the carrier from the internal oscillator to the sidechain input. |
| Mix           | 0–127 | MIX  | 0: no modulation (the input comes through at 0.99). 127: fully modulated. |
| Input         | 0–127 | INP  | Gain before the ring modulator, 4 × (Input / 128)²: 64 is unity, 127 is +11.9 dB, 0 is silence. |

The defaults are the machine's, which include Mix 0: it passes the signal
until you raise Mix. The machine's other four knobs (shown as "-" on the
unit) do nothing and are left out. All parameters can be automated and
modulated in Bitwig.

## How it behaves

This is what the firmware does; the plug-in keeps all of it.

- **The carrier.** An oscillator at the track's pitch, with a sine and a
  band-limited square (built from band-limited impulses at its edges and
  integrated) that are locked in phase. Wave crossfades the sine into the
  square through a low-pass that follows the pitch, then fades that low-pass
  out; a DC blocker, also following the pitch, sits after it. Ext crossfades
  all of it to the external input.
- **The modulation.** Each channel's input is multiplied by the carrier on two
  branches, one through a 15-sample delay and one through a pair of 8th-order
  allpasses that line up with that delay, and the two are averaged. Mix sets
  how much of the carrier, and how much of a plain 1, the input is multiplied
  by. In the allpass branch the channels cross: each output takes half the
  other channel's input. With a mono source this is an ordinary ring
  modulator; with a stereo one the two sides blend.
- **Pitch changes.** A new note is reached in two 16-sample blocks: 127/128 of
  the way, then the rest. That is the Monomachine's portamento at 0, as it
  is on an FX track.
- **Knobs.** Parameter changes reach the machine the way the Monomachine's
  CPU sends a knob: every 384 samples the value moves a quarter of the way to
  where you set it. Modulation is added on top, unsmoothed, every 48 samples,
  as the unit's LFOs are. Pitch and incoming notes take effect every 48
  samples.

## Differences from the hardware

- **The machine alone.** On the unit, RINGMOD is a machine on a track: it
  takes its input from INP A/B, a bus or the neighbouring track, and its
  output goes on through the track's amp, filter, EQ and delay. The plug-in
  is the machine alone: the host's audio goes in where the machine's input
  is, and the machine's output comes out.
- **Pitch.** On the unit the carrier follows the notes the track plays; here
  Pitch sets it, and incoming notes move it while Note Tracking is on. It is
  kept to the unit's notes, 0–127.
- **EXT.** On the unit, EXT takes the carrier from the external inputs, read
  one sample behind the machine's own input. The plug-in takes it from its
  sidechain, one sample behind the main input. That offset comes from the
  firmware's buffer positions. The emulator does not model those buffers, so
  it is not checked against them, though the EXT arithmetic itself is.
  Without a sidechain signal, Ext fades the carrier out.
- **Start.** The machine's first block after it is assigned reads its sine
  from an address outside its table. The plug-in starts with the sine at the
  table's start and runs 4 blocks of silence at every reset.
- **Other sample rates.** The machine runs at 44.1 kHz, the unit's rate. At
  other rates the plug-in resamples in and out (windowed sinc), so the output
  is no longer bit-exact, but the carrier stays at the same frequency.
- **Levels.** 0 dBFS in the plug-in is full scale at the machine's input.

Latency is 31 samples at 44.1 kHz: one 16-sample block, plus the machine's
own 15 samples, which the plug-in reports so the host can line it up.
It is 124 samples at 48 kHz and 246 at 96 kHz, exact to the sample.

## Accuracy

The reference is Monomachine OS 1.32B running in the DSP56300 emulator of
[Monomodule](https://github.com/shnolk/monomodule), with the machine's
controls and notes built by Monomodule's model of the Monomachine's CPU. It
has not been compared with a physical Monomachine.

| Check | Result |
|-------|--------|
| Core vs. firmware: random notes, knob moves through the CPU model, raw control words anywhere from 0 to $7FFFFF, inputs up to full scale and silence, about 403,000 blocks after each run's first | every output sample identical; every state word identical after every block; the frequency word identical to the kernel's in every block |
| Built plug-in code vs. firmware: knob moves through the CPU model, modulation, notes, Pitch and tracking changes and a sidechain (6,265 moves, 1,269 of them modulation and 1,254 notes, in 240,000 blocks) | every output sample identical |
| Tables vs. firmware memory | identical, every entry the machine reads |

The tables: the sine is the boot code's recurrence, reproduced exactly; the
exponential pitch table is 2^(22 + k/2048) with a mask for its 416 entries
one LSB lower in the firmware (as in
[Mono Dynamix](https://github.com/nerazin/mono-dynamix-clap)); the DC
blocker's table is the track filters' one-pole formula, exactly; the shaping
low-pass's is 1 − e^(−2πf/44100) for f = 4.003922 Hz × 2^(12.5 k / 255),
with five entries corrected; and the band-limited impulse is copied.

`tests/test_core.c` pins this without the firmware. Its expected hashes were
produced by feeding the same stimulus into the firmware's machine, which the
core matched.

## Build and install

```bash
./build_macos.sh
```

```bash
cp -R "build/Mono Ringmod.clap" ~/Library/Audio/Plug-Ins/CLAP/
```

The build script needs Xcode's command-line tools.

## Tests

```bash
cc -std=c99 -O2 -Wno-unused-function tests/test_core.c -lm -o tests/test_core && tests/test_core
```

```bash
xcrun clang -std=c11 -O2 -Wno-unused-function -Wno-missing-field-initializers -I vendor/clap/include tests/clap_test.c -o tests/clap_test && tests/clap_test "build/Mono Ringmod.clap"
```

`test_core` checks the tables and 12,000 blocks of output against the
firmware-derived hashes. `clap_test` loads the built plug-in and checks:

- at 44.1 kHz, output equal to the core's after its silent start, with the
  sidechain as EXT;
- with Mix at 0, the input arrives after the reported latency at 44.1, 48
  and 96 kHz;
- the carrier at 439.8 Hz for Pitch 69 at 44.1 and 48 kHz;
- a note moving the carrier with Note Tracking on, and not with it off;
- the sidechain as the carrier with Ext up;
- state and the parameter text.

The plug-in also passes clap-trap's `validate` (at 44.1, 48 and 96 kHz) and
`state --roundtrip`.

## Source

- `source/mono_ringmod.h`: the machine and the kernel's pitch glide (C99 and
  libm).
- `source/mono_ringmod_clap.c`: the CLAP plug-in around it.
- `vendor/clap`: CLAP 1.2 headers (MIT).

## License

GPL-3.0-only for this project's code. The DSP is an independent
reimplementation of the Monomachine's RINGMOD machine, derived from analysis
of Elektron's firmware; no firmware code is included, and the one table
copied from it is the oscillator's band-limited impulse. Monomachine and
Elektron are trademarks of Elektron Music Machines; this project is not
affiliated with Elektron.
