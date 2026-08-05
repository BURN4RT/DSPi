# DSPi Test Harness — Complete Guide

This is the end-to-end guide to the DSPi automated test harness, with a deep focus on
the **audio loopback** suite that measures real audio coming out of the device. It is
written so that someone who has never touched the project can wire it up, install the
tools, run the tests, understand every result, and fix common problems.

If you just want the short version, the quick-start README is
[`tools/dspi_test/README.md`](README.md). This document is the long, explain-everything
version.

---

## Table of contents

1. [What the harness is](#1-what-the-harness-is)
2. [The two test planes: control vs audio](#2-the-two-test-planes-control-vs-audio)
3. [What you need (hardware)](#3-what-you-need-hardware)
4. [Flashing the loopback firmware](#4-flashing-the-loopback-firmware)
5. [The DSPi loopback capture](#5-the-dspi-loopback-capture)
6. [Software setup](#6-software-setup)
7. [Running the tests](#7-running-the-tests)
8. [Command-line options](#8-command-line-options)
9. [How the audio measurement works](#9-how-the-audio-measurement-works)
9a. [Sample rates (48 kHz and 44.1 kHz)](#9a-sample-rates-48-khz-and-441-khz)
10. [Every test, explained](#10-every-test-explained)
11. [Every tunable parameter](#11-every-tunable-parameter)
12. [Reading the results](#12-reading-the-results)
13. [Troubleshooting](#13-troubleshooting)
14. [Scope and limits](#14-scope-and-limits)
15. [How it stays safe](#15-how-it-stays-safe)
16. [File map](#16-file-map)

---

## 1. What the harness is

The harness lives in [`tools/dspi_test/`](.) and is a host-side (your computer) program
that drives a connected DSPi over USB and checks that it behaves correctly. You run it
with Python from the repo root. It prints a pass/fail line per test and can write a
Markdown or JSON report.

There are two kinds of checks:

- **Control-plane checks** (the original suite): send every USB "vendor command" to the
  device and confirm it accepts/rejects values correctly and never crashes. These need
  only the DSPi plugged in.
- **Audio-loopback checks** (the functionality this guide is about): actually *play
  audio* into the DSPi, let its DSP process it, capture the result, and verify the sound
  matches what the firmware's math says it should be. These need DSPi flashed with the
  `DSPI_LOOPBACK` firmware build, which adds a USB capture input (described below).

---

## 2. The two test planes: control vs audio

| | Control plane | Audio loopback |
|---|---|---|
| What it proves | Commands round-trip, ranges validated, device stays alive | The DSP actually does the right thing to the audio |
| Hardware | DSPi only | DSPi flashed with the **DSPI_LOOPBACK** build (no extra hardware) |
| Python deps | `pyusb` | `pyusb` + `sounddevice` + `numpy` + `scipy` |
| Test group | everything except `audio` | the `audio` group |
| Runs by default? | yes | **no** — opt in with `--audio` |

The audio group is opt-in because it needs the special capture rig and extra
dependencies; on a machine without them it simply **skips** (it never fails the run).

---

## 3. What you need (hardware)

1. **A DSPi** flashed with the **`DSPI_LOOPBACK` firmware build**. It is both the device
   under test and the capture device: a USB sound card with an on-board DSP, where the
   loopback build adds a USB audio *input* that streams **output slot 0** back to the host
   (after all DSP). The normal/release firmware has no capture endpoint, so the audio group
   needs this special build (see [section 4](#4-flashing-the-loopback-firmware)).
   - Repo: <https://github.com/WeebLabs/DSPi>
   - With the loopback build it shows up as **"Weeb Labs DSPi"** in *both* directions — an
     output (playback) and a 2-channel 24-bit input (capture) — plus a USB vendor-control
     device with USB ID `2E8B:FEAA`.
2. **One USB cable** into the computer.

> No second board and no wiring are required: the capture is internal to DSPi (output slot 0
> is tapped in firmware). The harness matches the device **by name** ("DSPi" for both the
> output and the capture, disambiguated by direction), so you do not need device indices. If
> your OS names it differently, see [parameters](#11-every-tunable-parameter).

---

## 4. Flashing the loopback firmware

The audio group needs DSPi running the **`DSPI_LOOPBACK`** build. This is a debug build,
kept in its own build dirs so normal release builds are unaffected. From the repo root:

```bash
# Configure dedicated loopback build dirs (once):
cmake -S firmware -B build-rp2040-loopback -DPICO_PLATFORM=rp2040 -DPICO_BOARD=pico  -DDSPI_LOOPBACK=ON
cmake -S firmware -B build-rp2350-loopback -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2 -DDSPI_LOOPBACK=ON

# Build (pick your platform):
cmake --build build-rp2040-loopback -j     # RP2040
cmake --build build-rp2350-loopback -j     # RP2350
```

Flash the resulting `build-<plat>-loopback/DSPi/DSPi.uf2`: hold **BOOTSEL** while plugging
DSPi in (it appears as a USB drive), then copy the UF2 onto it. DSPi reboots and now
enumerates with **both** a playback output and a 2-channel capture input, both named
"Weeb Labs DSPi".

> The loopback feature is gated behind the `DSPI_LOOPBACK` flag and is excluded from normal
> release builds, so a production DSPi will not expose the capture input. See
> `Documentation/current_architecture.md` ("USB Audio Loopback Capture") for the firmware
> details.

The signal path the tests exercise is entirely internal — no S/PDIF cable:

```
   your computer                       DSPi (DSPI_LOOPBACK build)                 your computer
  ┌──────────────┐   USB         ┌────────────────────────────────┐   USB       ┌──────────────┐
  │  test script │ ──────────►   │ USB-audio in → DSP → slot 0     │ ──────────► │  records the │
  │ plays a sweep│  (audio out)  │ tapped after DSP → USB capture  │  (audio in) │  captured    │
  │              │               │                                 │             │  audio       │
  └──────────────┘               └────────────────────────────────┘             └──────────────┘
```

---

## 5. The DSPi loopback capture

The loopback capture is what makes audio testing possible — it gives the host a way to
record what DSPi produced. In the `DSPI_LOOPBACK` build it is **built into DSPi itself**.

- **What it is:** a second USB Audio Class 1 function on DSPi that streams **output slot 0**
  back to the host as **Type-I PCM, 2 channels, 24-bit, 44.1 / 48 kHz** on an isochronous IN
  endpoint. (Earlier revisions of this rig used a separate "Weeb Labs USBrx" board wired to a
  DSPi S/PDIF output; that is no longer needed.)
- **Where it taps:** output slot 0's finished samples, read *after* all DSP (EQ, crossover,
  matrix, gain, delay, mute) and *before* the output encoder. So it captures exactly what
  slot 0 sends out, whether slot 0 is configured as S/PDIF or I2S.
- **Read-only:** the tap copies slot 0's samples; it never modifies them, so it cannot
  disturb the firmware's inter-slot output alignment.

### Clock note (why the measurement is so accurate)

DSPi is the timing master. The host's playback to DSPi is rate-locked to DSPi by USB audio
feedback, and the capture is slaved to DSPi's output rate via a small rate-matching servo. So
everything runs on **one clock** and the captured samples are a fixed-latency, near bit-exact
copy of slot 0. That is why the magnitude measurements land within ~0.00 dB of the math.

### Polarity

Unlike the old S/PDIF-receiver rig (whose decoder inverted polarity), the integrated capture
is a **direct digital copy** of slot 0, so it is **not** polarity-inverted. The harness is
polarity-agnostic regardless: magnitude checks ignore sign, phase checks remove a constant
offset, and the phase-invert test only checks that the sign *flips* — so either polarity passes.

---

## 6. Software setup

You need Python 3 and a few packages. Run everything **from the repo root**
(`/path/to/DSPi`) so the harness can find the firmware's command list in `config.h`.

```bash
# 1. USB control (always needed)
pip install pyusb
#    macOS also needs the native USB library:
brew install libusb

# 2. Audio measurement (needed for the `audio` group)
pip install sounddevice numpy scipy
#    sounddevice bundles PortAudio; on some Linux distros you may also need:
#    sudo apt install libportaudio2
```

> If your Python is "externally managed" (Homebrew/PEP 668) and `pip install` is refused,
> either use a virtual environment, or `pip install --user ...`, or (matching how this
> machine's deps were installed) `pip install --break-system-packages ...`.

### macOS microphone permission

macOS treats any USB audio **input** (including the DSPi capture) as a microphone. The **first
time** you run an audio test, macOS will prompt to allow your terminal app to access the
microphone — **allow it**. If you miss the prompt, enable it under
*System Settings → Privacy & Security → Microphone* for your terminal/Python, then re-run.
Until granted, captures come back as silence and the audio tests will report "no signal".

---

## 7. Running the tests

All commands are run from the repo root.

**Just want to run everything?** This one command is the complete suite — it runs the
audio-loopback group **and** the flash and factory-reset tests, and needs no other
knowledge:

```bash
python3 -m tools.dspi_test.run --all
```

`--all` is exactly `--audio --allow-flash --allow-factory-reset`, and it runs the **audio
group first** so its auto-probe sees a pristine device (before the control-plane tests
change device state). If the loopback firmware or the audio libraries are missing, the audio
tests **skip** (they never fail the run). A full pass looks like:

```
RESULT: 131/133 PASS · 0 FAIL · 0 ERROR · 2 SKIP
```

(The two skips are by design: the "enter bootloader" test is policy-excluded so it cannot
end your session, and one preset test stops once the flash-erase budget is reached — raise
it with `--flash-cap N` if you want that one too.)

Other ways to run it:

```bash
# Control-plane suite only (safe, fast, no special hardware):
python3 -m tools.dspi_test.run

# Include the audio loopback group only (no flash / no factory reset):
python3 -m tools.dspi_test.run --audio

# Run ONLY the audio group:
python3 -m tools.dspi_test.run --group audio

# Complete suite + reports:
python3 -m tools.dspi_test.run --all --report report.md --json report.json
```

### Bring-up / first-light checks

Before running the suite, sanity-check the rig with the standalone audio tool:

```bash
# 1. List every audio device the computer sees (confirm DSPi appears as both
#    an output and an input):
python3 -m tools.dspi_test.audio --list

# 2. Play a 1 kHz tone out the DSPi and read it back from the DSPi capture,
#    printing level / THD / noise (a quick capture sanity check):
python3 -m tools.dspi_test.audio --probe
```

If `--list` does not show **"Weeb Labs DSPi"** as both an output **and** an input, fix that
first (cable, drivers, the `DSPI_LOOPBACK` firmware) before running the suite.

A normal full audio run ends with something like:

```
RESULT: 40/40 PASS · 0 FAIL · 0 ERROR · 0 SKIP
```

and restores the device to exactly the state it was in before the run.

---

## 8. Command-line options

`python3 -m tools.dspi_test.run [options]`

| Flag | What it does |
|---|---|
| `--all` | **Complete suite in one command:** audio loopback + flash + factory reset (= `--audio --allow-flash --allow-factory-reset`). Audio runs first. |
| `--audio` | Include the `audio` loopback group (excluded by default). |
| `--group G[,G...]` | Run only these groups, e.g. `--group audio` or `--group eq,volume`. |
| `--list` | List registered tests and exit (no device needed). Add `--audio` to also list audio tests. |
| `--report PATH.md` | Write a Markdown report (per-test results + the measured-margin notes). |
| `--json PATH.json` | Write a machine-readable JSON report. |
| `--quiet` | Suppress the per-test console lines. |
| `--allow-flash` | Enable control-plane tests that write flash (not used by the audio group). |
| `--allow-factory-reset` | Enable the one-shot factory-reset control-plane test. |
| `--catalog PATH.md` | Write the auto-generated test catalog and exit. |

`python3 -m tools.dspi_test.audio [--list] [--probe] [--out-name NAME] [--in-name NAME] [--fs HZ] [--channel N]`

| Flag | What it does |
|---|---|
| `--list` | Enumerate host audio devices. |
| `--probe` | Play a 1 kHz tone to the DSPi and read it back; print level/THD/noise. |
| `--out-name` | Substring to match the DSPi output device (default `DSPi`). |
| `--in-name` | Substring to match the DSPi capture input device (default `DSPi`). |
| `--fs` | Sample rate in Hz (default 48000). |
| `--channel` | Which slot-0 channel to read (0 = left, 1 = right). |

Exit code is `0` only if there were no failures or errors.

---

## 9. How the audio measurement works

Understanding this makes every test and parameter obvious.

1. **Auto-probe (once per run).** The harness plays a tone on each S/PDIF slot in turn
   (isolating one at a time) and watches which one reaches the capture. With the integrated
   tap this is always slot 0; the probe confirms it empirically. That slot becomes the
   "target" for the rest of the run.
   The audio group always runs **before** the control-plane tests, so this probe happens
   on a pristine device (this is why `--all` works in a single command).

2. **Configure a clean path.** For the target slot the harness sets: input source = USB,
   master/user volume and preamp = 0 dB, the output enabled and set to S/PDIF, the USB
   L/R routed 1:1 to the slot at 0 dB, and all EQ bands flat. So unless a test changes one
   thing on purpose, the path is unity.

3. **Play, capture, align.** A test plays an excitation (a logarithmic sine **sweep** for
   frequency response, or a steady **tone** for levels) out the DSPi and records the DSPi
   capture at the same time using two independent audio streams. Because the chain is single-clock,
   the recording is just a delayed copy; the harness finds that delay by **cross-correlation**
   and lines the recording up with what it played.

4. **Compute the result.**
   - *Frequency response:* divide the captured spectrum by the played spectrum to get the
     filter's transfer function `H(f)`, then compare its magnitude (and, for all-pass, its
     phase) to the **expected** response computed from the firmware's own filter math
     (`tools/filter_tester/compare_filter.py` for PEQ; scipy for crossovers).
   - *Levels:* measure the RMS level of the captured tone in dBFS.
   - *Delay/alignment:* cross-correlate the two captured channels to get their sample
     offset.
   - *Fidelity:* fit the best single gain between the captured sweep and what was played;
     the leftover (residual) shows how close to bit-exact the path is.

Because the digital path is essentially perfect, the **measured error margins are tiny**
(magnitude errors around 0.00 dB), and the pass thresholds are set far looser than that so
real measurement noise never trips them.

---

## 9a. Sample rates (48 kHz and 44.1 kHz)

DSPi has no vendor command for its operating rate: it follows the host USB rate.
The loopback capture function advertises 44.1 and 48 kHz only, so both are real
operating points and both are worth measuring. The playback function also
advertises 96 kHz, but the capture cannot carry it (the servo's per-frame packet
ceiling is 52 stereo frames), so the harness skips rather than measuring garbage
there.

**Changing the rate needs the CoreAudio HAL, not a stream open.** Opening a
PortAudio stream at 44.1 kHz appears to work and reports the stream running at
44100, but macOS leaves the device at its nominal rate and resamples in
software. `tools/dspi_test/coreaudio.py` sets
`kAudioDevicePropertyNominalSampleRate` directly instead. The loopback build
exposes two UAC functions, so macOS creates two devices (playback and capture)
and **both** must move, or `play_record` straddles a rate boundary and CoreAudio
resamples one side. Off macOS the helper is a no-op and the 44.1 kHz tests skip.

**What runs at each rate.** By default the **full matrix runs at both rates**
(175 tests, about 9 minutes). Only one configuration in the test tables actually
changes DSP path between them, since the hybrid SVF/biquad boundary is Fs/7.5 and
moves from 6400 Hz to 5880 Hz: `multiband_eq`'s 6 kHz high shelf is on the SVF
side at 48 kHz and the biquad side at 44.1 kHz. Every other filter runs the same
code at both rates, but its coefficients are recomputed from fs, so a
rate-dependent fault (a wrong fs normalisation, a clamp against the wrong rate)
would show at 44.1 kHz only. Running everything at both costs about 3.5 minutes
over a single-rate pass, which is cheap enough not to economise on.

A few behaviours are unique to 44.1 kHz and worth knowing:

| Behaviour | Why it only appears at 44.1 kHz |
|---|---|
| Capture servo fractional accumulator | 44.1 frames per USB frame is non-integer; at 48 kHz the accumulator sits idle and only the feed-forward term runs |
| Output-delay sample count | derived from fs: 220 samples for 5 ms, versus 240 at 48 kHz |
| Hybrid boundary crossing | `multiband_eq`'s 6 kHz shelf, as above |

`--audio-rates` narrows the set, e.g. `--audio-rates 48000` for a faster
single-rate pass (88 tests). Variants are registered in rate order with
`rate_switch_round_trip`
last, so a run performs one rate change per extra rate rather than thrashing,
and the device is left on the primary rate at the end. The rate is host-driven
and absent from the bulk blob, so the suite's snapshot cannot restore it; that
final test is what puts it back.

---

## 10. Every test, explained

All audio tests are in the `audio` group. "PASS" means the firmware's real audio output
matched the expectation within the listed tolerance.

### Baseline

| Test | What it does | Passes when |
|---|---|---|
| `loopback_integrity` | Plays a tone/sweep through a flat (no-effect) path. | Signal arrives at unity level, the noise floor is very low, THD ≈ 0, and the captured audio is a near bit-exact copy of what was played (the path gain magnitude is ≈ 1). |

### Parametric EQ (per-band filters)

| Test | What it does | Passes when |
|---|---|---|
| `peq_peaking_lo/hi/cut`, `peq_lowshelf`, `peq_highshelf`, `peq_lowpass`, `peq_highpass`, `peq_notch`, `peq_lowshelf1`, `peq_highshelf1` | Sets one PEQ band to a given type/frequency/Q/gain on the output, sweeps, and compares the measured magnitude curve to the RBJ-cookbook reference. Configs span both sides of the RP2350 SVF/biquad boundary (~6.4 kHz). | Max magnitude error < `MAG_TOL_DB` (0.7 dB). |
| `loopback_allpass_phase` | Sets a first-order all-pass and checks it does not change magnitude but does rotate phase the expected way. | Magnitude stays flat and the phase shape matches (after removing a constant delay). |

### Crossover filters

| Test | What it does | Passes when |
|---|---|---|
| `xo_lr2_lp` … `xo_bes8_lp` (13 configs) | Sets a crossover type (Linkwitz-Riley, Butterworth, Bessel; orders 1–8; low-pass and high-pass) on a crossover band, sweeps, and compares to the scipy reference. | Max magnitude error < `XO_MAG_TOL_DB` (1.0 dB) over the measurable region (where the response is above `XO_MAG_FLOOR_DB` = −60 dB). |
| `xo_lr4_complementary_sum` | Puts LR4 low-pass on one leg and LR4 high-pass on the other, measured in one capture, and adds them. | Their sum is flat (the defining Linkwitz-Riley property) within 1 dB. |

### Output-stage controls

| Test | What it does | Passes when |
|---|---|---|
| `output_gain_level` | Sets the per-output gain to several dB values. | Measured level changes by the set dB (±`LEVEL_TOL_DB` = 0.5 dB). |
| `output_mute_silences` | Mutes, then un-mutes the output. | Muted → below `MUTE_FLOOR_DBFS` (−80 dBFS); un-mute restores the level. |
| `level_controls` | Sets master volume, user (host) volume, and per-input preamp each to −6 dB. | Each scales the measured level by −6 dB (±0.5 dB). |
| `matrix_routing` | Disables, then re-enables, a matrix crosspoint. | Routed → signal present; unrouted → silent (< −80 dBFS); re-route restores. |
| `matrix_phase_invert` | Toggles the crosspoint phase-invert flag. | The fitted path-gain sign flips (e.g. −1.0 → +1.0). |
| `output_delay` | Sets a 5 ms per-output delay on one leg vs an undelayed leg. | The two legs differ by exactly the set sample count (240 @ 48 kHz, ±1). |

### Alignment / latency stability

| Test | What it does | Passes when |
|---|---|---|
| `slot_lr_alignment` | Measures the captured slot's L vs R sample offset. | L and R are sample-aligned (offset ≤ `ALIGN_TOL_SAMPLES` = 1). |
| `alignment_after_input_switch` | Switches input USB → S/PDIF → USB. | Signal returns and L/R are still aligned. |
| `alignment_after_output_type_switch` | Switches the slot S/PDIF → I2S → S/PDIF. | Signal returns and L/R are still aligned (skips if I2S switch is unavailable). |

> These verify **intra-slot** L/R alignment and that the firmware's pipeline-reset
> operations preserve it. Verifying alignment *between different slots* (the firmware's
> full guarantee) needs a multi-channel capture and is out of scope for one stereo capture.

### Output types (S/PDIF and I2S)

| Test | What it does | Passes when |
|---|---|---|
| `output_type_i2s_audio` | Sets slot 0 to **I2S** output and measures a real tone. | Tone reaches the capture at unity, THD < 0.1%, near bit-exact, L/R aligned (skips if I2S unavailable). |
| `output_type_switch_stress` | Cycles slot 0 S/PDIF↔I2S four times, re-measuring after every switch. | Signal present, at unity level, and L/R aligned after each of the 8 switches (skips if I2S unavailable). |

> The loopback tap is **pre-encoder**, so the captured samples are identical for either
> output type; signal *presence* is therefore a liveness probe for the active output path
> (a dead I2S consumer would stall slot 0's producer and silence the capture).
> `output_type_switch_stress` is the hardware regression test for the shared-DMA-channel
> teardown/re-setup path (S/PDIF and I2S TX share one DMA channel per output slot): a
> double-claim, a stalled pipeline, or lost alignment surfaces as lost signal, a bad lag,
> or a device reset.

### Full chain / dynamics

| Test | What it does | Passes when |
|---|---|---|
| `multiband_eq` | Stacks three PEQ bands (shelf + cut + shelf) at once. | The combined response equals the sum (in dB) of the individual bands (< 0.7 dB). |
| `loudness_shape` | Enables loudness at a low (−40 dB) volume. | Bass and treble are boosted relative to the midrange (the equal-loudness contour). |
| `crossfeed_bleed` | Plays one channel only, with crossfeed off then on. | Off → opposite channel silent; on → an attenuated, filtered copy bleeds into it. |
| `leveller_boost` | Plays a quiet tone with the leveller off then on. | The leveller lifts the quiet signal, by more than a few dB but within the max-gain ceiling. |
| `output_clip_limit` | Boosts a near-full-scale tone past 0 dBFS with a +12 dB filter. | The output clamps at full scale (does not wrap) and THD rises sharply. |

---

## 11. Every tunable parameter

These are constants at the top of the files; change them only if you understand the
trade-off. Values shown are the defaults.

### Pass/fail tolerances — `tools/dspi_test/tests/audio_loopback.py`

| Constant | Default | Meaning |
|---|---|---|
| `MAG_TOL_DB` | `0.7` | Max allowed magnitude error for PEQ frequency-response tests, in dB. |
| `PHASE_TOL_DEG` | `6.0` | Max allowed phase-shape error for the all-pass test, in degrees (after removing a constant delay). |
| `CORR_MIN` | `0.30` | Minimum cross-correlation strength counted as "signal present". |
| `NOISE_MAX_DBFS` | `-100.0` | Integrity test: noise floor must be quieter than this. |
| `RESIDUAL_MAX_DBFS` | `-80.0` | Integrity test: flat-path per-sample residual must be quieter than this (near bit-exact). |
| `GAIN_TOL_DB` | `0.5` | Integrity test: how close the flat-path gain magnitude must be to unity (0 dB). |
| `XO_MAG_TOL_DB` | `1.0` | Max magnitude error for crossover tests (steeper than PEQ, so a touch looser). |
| `XO_MAG_FLOOR_DB` | `-60.0` | Crossover tests only compare where the response is above this (deep stopbands are not reliably measurable). |
| `LEVEL_TOL_DB` | `0.5` | How closely a measured level change must match a set gain/volume/preamp dB. |
| `MUTE_FLOOR_DBFS` | `-80.0` | A muted / unrouted output must be quieter than this. |
| `ALIGN_TOL_SAMPLES` | `1` | Max allowed sample offset for "aligned". |
| `FS` | `48000` | Sample rate used for the audio tests (the host streams and DSPi follow it). |

### Test content (what gets swept) — `audio_loopback.py`

- `PEQ_CONFIGS` — the list of `(name, type, freq, Q, gain)` PEQ points tested.
- `XO_CONFIGS` — the list of `(name, family, order, is_hp, fc)` crossover points tested.
- Add or remove rows here to broaden or narrow coverage.

### Engine knobs — `tools/dspi_test/audio.py`

| Constant / arg | Default | Meaning |
|---|---|---|
| `DSPI_OUT_NAME` | `"DSPi"` | Substring used to find the DSPi output device by name. |
| `DSPI_IN_NAME` | `"DSPi"` | Substring used to find the DSPi capture input device by name. |
| `DEFAULT_FS` | `48000` | Default sample rate. |
| `PAD_S` | `0.10` | Leading/trailing silence (seconds) around each excitation. |
| `TAIL_S` | `0.40` | Extra recording time after playback, to capture path latency + decay. |
| `play_record(..., max_retries=3)` | `3` | If a capture has an audio dropout (xrun), retry up to this many times. |

If your OS shows the devices under different names, change `DSPI_OUT_NAME` /
`DSPI_IN_NAME` (or pass `--out-name` / `--in-name` to the `audio` tool).

---

## 12. Reading the results

- **Console:** one line per test — `✓` pass, `✗` fail, `–` skip, `!` error.
- **Notes / measured margins:** each audio test records the actual numbers it measured
  (e.g. `peq_peaking_lo: max_mag_err=0.000dB`). These appear in the "Notes & observations"
  section of the Markdown report (`--report`). Tiny margins (≈0.00 dB) are normal and
  expected for the digital path.
- **SKIP** on an audio test means the capture or deps were missing (e.g. DSPi not on the
  `DSPI_LOOPBACK` build, sounddevice not installed, or microphone permission not granted) —
  not a failure.
- After every run the harness prints whether it restored the device to its pre-run state
  ("live state restored byte-for-byte").

---

## 13. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Audio tests all **SKIP** | Missing deps (`pip install sounddevice numpy scipy`) or the DSPi output/capture not found by name. Run `python3 -m tools.dspi_test.audio --list`. |
| `--list` shows the DSPi output but **no DSPi input** | DSPi is not running the `DSPI_LOOPBACK` build (release firmware has no capture endpoint). Re-flash with the loopback UF2; or set `--in-name` to a matching substring. |
| Captures are **silent** / "no signal" / probe shows very low level | (1) macOS microphone permission not granted — allow it and retry. (2) Bare `--probe` does not configure routing — use the suite (which sets input=USB and routes to slot 0), or route USB → slot 0 first. |
| `no output slot reached the DSPi capture` | DSPi input source is not USB, or output slot 0 is muted/unrouted. The suite configures these; check you are not relying on the bare `--probe`. |
| Results are **erratic / low correlation** | An audio dropout (xrun). The harness already retries; if it persists, close other audio apps, or increase `play_record(max_retries=...)`. |
| Magnitude off by a constant on **every** XO/PEQ test | A leftover filter on a band the test does not flatten, or the path is not unity (check master/user volume, preamp). The tests flatten the chain themselves, so this usually means a custom change. |
| `pip install` refused ("externally managed") | Use a venv, `--user`, or `--break-system-packages` (see [setup](#6-software-setup)). |
| Polarity shows inverted (`scale = -1`) | Expected — it is the S/PDIF receive path, not a bug. The tests are polarity-agnostic and still pass. |

---

## 14. Scope and limits

What the single-stereo-capture rig **can** verify (and does): per-band PEQ and crossover
frequency response, all-pass phase, output gain/mute/volume/preamp/routing/phase/delay,
intra-slot L/R alignment and its stability across pipeline resets, multiband EQ, loudness,
crossfeed, leveller, and clipping behavior. Because the tap is before the output encoder,
slot 0 is captured whether it is configured as S/PDIF or I2S.

What it **cannot** verify with this rig:

- **Inter-slot alignment** between the different S/PDIF slots / I2S / PDM (the firmware's
  full alignment guarantee) — needs a multi-channel digital capture or a second receiver.
- **Other output slots and the PDM sub** — the capture taps only slot 0; slots 1-3 and PDM
  are not captured.
- **The S/PDIF / I2S hardware encoders themselves** — the tap is the pre-encode sample
  stream, so it validates the DSP, not the line-coding of those outputs.
- **Sample-rate sweeps** — the suite runs at 48 kHz by design.

---

## 15. How it stays safe

- The whole audio group is **opt-in** (`--audio`) and **skips** cleanly when the rig/deps
  are absent, so it never breaks a normal control-plane run.
- It writes **no flash** — every change is to live RAM state.
- Each test restores what it changed, and the runner takes a **full snapshot before the
  run and restores it after**, confirming "live state restored byte-for-byte".
- The test signals are digital, captured internally by DSPi; nothing is played on speakers.

---

## 16. File map

| File | Role |
|---|---|
| `tools/dspi_test/run.py` | CLI entry point and runner (`python3 -m tools.dspi_test.run`). |
| `tools/dspi_test/framework.py` | Test registry, the `Check` assertions, the serial runner, report output. |
| `tools/dspi_test/device.py` | USB vendor-command transport to the DSPi (`pyusb`). |
| `tools/dspi_test/profile.py` | Detects the attached board's capabilities (channels, slots, etc.). |
| `tools/dspi_test/lifecycle.py` | Pre-run snapshot and post-run restore. |
| `tools/dspi_test/selftest.py` | No-hardware checks of the harness's own logic (`python3 -m tools.dspi_test.selftest`). |
| `tools/dspi_test/audio.py` | The audio measurement engine (device discovery, play/record, sweeps, metrics) + the `--list`/`--probe` tool. |
| `tools/dspi_test/coreaudio.py` | macOS HAL access to set the device's nominal sample rate (PortAudio cannot: it resamples instead). No-op off macOS. |
| `tools/dspi_test/tests/audio_loopback.py` | The `audio` group: the loopback tests + their fixtures and parameters. |
| `tools/dspi_test/tests/*.py` | The control-plane test modules (eq, outputs, inputs, volume, presets, etc.). |
| `tools/filter_tester/compare_filter.py` | RBJ filter reference reused for the expected PEQ responses. |
| `tools/dspi_test/README.md` | The short quick-start (this guide is the long version). |

---

*Related repos:* [DSPi](https://github.com/WeebLabs/DSPi)
(the [USBrx](https://github.com/WeebLabs/USBrx) external recorder is superseded by the
built-in `DSPI_LOOPBACK` capture).
