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
4. [Wiring the loopback rig](#4-wiring-the-loopback-rig)
5. [The USBrx capture device](#5-the-usbrx-capture-device)
6. [Software setup](#6-software-setup)
7. [Running the tests](#7-running-the-tests)
8. [Command-line options](#8-command-line-options)
9. [How the audio measurement works](#9-how-the-audio-measurement-works)
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
- **Audio-loopback checks** (the new functionality this guide is about): actually *play
  audio* into the DSPi, let its DSP process it, capture the result, and verify the sound
  matches what the firmware's math says it should be. These need the extra capture rig
  described below.

---

## 2. The two test planes: control vs audio

| | Control plane | Audio loopback |
|---|---|---|
| What it proves | Commands round-trip, ranges validated, device stays alive | The DSP actually does the right thing to the audio |
| Hardware | DSPi only | DSPi **+ Weeb Labs USBrx** wired to a DSPi S/PDIF output |
| Python deps | `pyusb` | `pyusb` + `sounddevice` + `numpy` + `scipy` |
| Test group | everything except `audio` | the `audio` group |
| Runs by default? | yes | **no** — opt in with `--audio` |

The audio group is opt-in because it needs the special capture rig and extra
dependencies; on a machine without them it simply **skips** (it never fails the run).

---

## 3. What you need (hardware)

1. **A DSPi.** This is the device under test: a USB sound card with an on-board DSP that
   outputs S/PDIF (and/or I2S) plus a PDM subwoofer channel.
   - Repo: <https://github.com/WeebLabs/DSPi>
   - It shows up to the computer as a USB audio device named **"Weeb Labs DSPi"** and as a
     USB vendor-control device with USB ID `2E8B:FEAA`.
2. **A Weeb Labs USBrx.** This is the capture device: it receives an S/PDIF signal and
   re-presents it to the computer as a USB audio *input* (like a microphone), so the host
   can record what the DSPi sent out.
   - Repo: <https://github.com/WeebLabs/USBrx>
   - It shows up as a USB audio input named **"Weeb Labs USBrx"** (24-bit, 2-channel).
3. **Two USB cables** (one per device) into the same computer.
4. **Two jumper wires** to connect a DSPi S/PDIF output to the USBrx S/PDIF input (see
   next section).

> The harness matches the two devices **by name** ("DSPi" for output, "USBrx" for input),
> so you do not need to know device indices. If your OS names them differently, see
> [parameters](#11-every-tunable-parameter).

---

## 4. Wiring the loopback rig

The signal path the tests exercise is:

```
   your computer                 DSPi                          USBrx              your computer
  ┌──────────────┐   USB     ┌──────────────┐   S/PDIF     ┌──────────────┐   USB   ┌──────────────┐
  │  test script │ ───────►  │  USB-audio   │ ──wire────►  │  S/PDIF RX    │ ──────► │  records the │
  │ plays a sweep│  (audio   │  in → DSP →  │  (GPIO to    │  → USB-audio  │ (audio  │  captured    │
  │              │   out)    │  S/PDIF out  │   GPIO)      │  in (24-bit)  │   in)   │  audio       │
  └──────────────┘           └──────────────┘              └──────────────┘         └──────────────┘
```

Make the single hardware connection:

| From: DSPi S/PDIF output GPIO | To: USBrx S/PDIF input GPIO |
|---|---|
| any S/PDIF output pin (default **GP6** = slot 0; also GP7/8/9 on RP2350) | **GP15** |
| **GND** | **GND** (common ground is required) |

Notes for beginners:

- DSPi drives S/PDIF as a **3.3 V logic-level** square-wave signal straight off a GPIO
  pin, and the USBrx reads it on a GPIO. So a **direct jumper wire** (GPIO → GP15) plus a
  **shared ground** is all you need on the bench. You do not need a coax/optical S/PDIF
  transceiver for testing.
- You can wire to **any** of the DSPi S/PDIF outputs. The harness **auto-detects** which
  one the USBrx is on (it plays a tone on each and watches which one shows up). The
  default GPIO for the first S/PDIF output (slot 0) is **GP6**.
- Keep the wire short. It is a fast digital signal.

---

## 5. The USBrx capture device

The USBrx is what makes audio testing possible — the DSPi has no audio *input* back to
the computer, so without USBrx the harness could only check commands, not sound.

- **What it is:** a Raspberry Pi Pico (RP2040) running the firmware at
  <https://github.com/WeebLabs/USBrx>. It decodes incoming S/PDIF and streams it to the
  host as a standard USB Audio Class 1 input: **Type-I PCM, 2 channels, 24-bit, 44.1 /
  48 / 88.2 / 96 kHz**.
- **S/PDIF decoding** is done by the `pico_spdif_rx` PIO library
  (<https://github.com/elehobica/pico_spdif_rx>), included as a submodule.
- **S/PDIF input pin:** GP15 (`SPDIF_DATA_PIN` in `src/main.c`).

### Building and flashing the USBrx (one-time)

You only do this once, to put the firmware on the Pico. From a terminal:

```bash
git clone --recurse-submodules https://github.com/WeebLabs/USBrx.git
cd USBrx
# The repo pins its own pico-sdk; if you cloned without --recurse-submodules:
git submodule update --init --recursive
mkdir build && cd build
cmake ..
make -j
```

This produces `usbrx.uf2` in the build directory. To flash it: hold the Pico's **BOOTSEL**
button while plugging it in (it appears as a USB drive), then copy `usbrx.uf2` onto that
drive. The Pico reboots and enumerates as **"Weeb Labs USBrx"**. See the USBrx repo's own
[README](https://github.com/WeebLabs/USBrx) for the latest details.

### Clock note (why the measurement is so accurate)

DSPi is the timing master. The host's playback to DSPi is rate-locked to DSPi by USB
audio feedback, and the USBrx is locked to DSPi's S/PDIF. So everything runs on **one
clock** and the captured samples are a fixed-latency, bit-exact copy of DSPi's output.
That is why the magnitude measurements land within ~0.00 dB of the math.

### One known quirk: polarity

The `pico_spdif_rx` decoder reconstructs audio that is **polarity-inverted** relative to
what DSPi sent (a property of how that decoder handles S/PDIF line polarity; it is *not* a
bug in DSPi or in the test code). The harness is deliberately **polarity-agnostic**:
magnitude checks ignore sign, and phase checks remove a constant offset, so the inversion
never causes a false failure.

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

macOS treats any USB audio **input** (including the USBrx) as a microphone. The **first
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
change device state). If the USBrx rig or the audio libraries are missing, the audio tests
**skip** (they never fail the run). A full pass looks like:

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
# 1. List every audio device the computer sees (confirm DSPi + USBrx appear):
python3 -m tools.dspi_test.audio --list

# 2. Play a 1 kHz tone out the DSPi and read it back from the USBrx,
#    printing level / THD / noise (a quick "is the wire connected?" test):
python3 -m tools.dspi_test.audio --probe
```

If `--list` does not show both **"Weeb Labs DSPi"** (an output) and **"Weeb Labs USBrx"**
(an input), fix that first (cables, drivers, firmware) before running the suite.

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
| `--in-name` | Substring to match the USBrx input device (default `USBrx`). |
| `--fs` | Sample rate in Hz (default 48000). |
| `--channel` | Which USBrx channel to read (0 = left, 1 = right). |

Exit code is `0` only if there were no failures or errors.

---

## 9. How the audio measurement works

Understanding this makes every test and parameter obvious.

1. **Auto-probe (once per run).** The harness plays a tone on each S/PDIF slot in turn
   (isolating one at a time) and watches which one reaches the USBrx. That slot becomes
   the "target" for the rest of the run. This is why you can wire to any S/PDIF output.
   The audio group always runs **before** the control-plane tests, so this probe happens
   on a pristine device (this is why `--all` works in a single command).

2. **Configure a clean path.** For the target slot the harness sets: input source = USB,
   master/user volume and preamp = 0 dB, the output enabled and set to S/PDIF, the USB
   L/R routed 1:1 to the slot at 0 dB, and all EQ bands flat. So unless a test changes one
   thing on purpose, the path is unity.

3. **Play, capture, align.** A test plays an excitation (a logarithmic sine **sweep** for
   frequency response, or a steady **tone** for levels) out the DSPi and records the USBrx
   at the same time using two independent audio streams. Because the chain is single-clock,
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
> full guarantee) needs a multi-channel capture and is out of scope for one stereo USBrx.

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
| `USBRX_IN_NAME` | `"USBrx"` | Substring used to find the USBrx input device by name. |
| `DEFAULT_FS` | `48000` | Default sample rate. |
| `PAD_S` | `0.10` | Leading/trailing silence (seconds) around each excitation. |
| `TAIL_S` | `0.40` | Extra recording time after playback, to capture path latency + decay. |
| `play_record(..., max_retries=3)` | `3` | If a capture has an audio dropout (xrun), retry up to this many times. |

If your OS shows the devices under different names, change `DSPI_OUT_NAME` /
`USBRX_IN_NAME` (or pass `--out-name` / `--in-name` to the `audio` tool).

---

## 12. Reading the results

- **Console:** one line per test — `✓` pass, `✗` fail, `–` skip, `!` error.
- **Notes / measured margins:** each audio test records the actual numbers it measured
  (e.g. `peq_peaking_lo: max_mag_err=0.000dB`). These appear in the "Notes & observations"
  section of the Markdown report (`--report`). Tiny margins (≈0.00 dB) are normal and
  expected for the digital path.
- **SKIP** on an audio test means the rig or deps were missing (e.g. no USBrx, sounddevice
  not installed, or microphone permission not granted) — not a failure.
- After every run the harness prints whether it restored the device to its pre-run state
  ("live state restored byte-for-byte").

---

## 13. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Audio tests all **SKIP** | Missing deps (`pip install sounddevice numpy scipy`) or the USBrx/DSPi not found by name. Run `python3 -m tools.dspi_test.audio --list`. |
| `--list` shows DSPi but **no USBrx** | USBrx not plugged in, not flashed, or named differently. Re-flash it; or set `--in-name` to a matching substring. |
| Captures are **silent** / "no signal" / probe shows very low level | (1) macOS microphone permission not granted — allow it and retry. (2) The S/PDIF wire or ground is not connected. (3) Wrong DSPi output pin — but the harness auto-probes all slots, so usually it is the wire/ground or permission. |
| `no S/PDIF slot reached USBrx` | The wire from a DSPi S/PDIF output GPIO to USBrx **GP15** (and a common GND) is missing or on the wrong pin. |
| Results are **erratic / low correlation** | An audio dropout (xrun). The harness already retries; if it persists, close other audio apps, or increase `play_record(max_retries=...)`. |
| Magnitude off by a constant on **every** XO/PEQ test | A leftover filter on a band the test does not flatten, or the path is not unity (check master/user volume, preamp). The tests flatten the chain themselves, so this usually means a custom change. |
| `pip install` refused ("externally managed") | Use a venv, `--user`, or `--break-system-packages` (see [setup](#6-software-setup)). |
| Polarity shows inverted (`scale = -1`) | Expected — it is the S/PDIF receive path, not a bug. The tests are polarity-agnostic and still pass. |

---

## 14. Scope and limits

What the single-stereo-USBrx rig **can** verify (and does): per-band PEQ and crossover
frequency response, all-pass phase, output gain/mute/volume/preamp/routing/phase/delay,
intra-slot L/R alignment and its stability across pipeline resets, multiband EQ, loudness,
crossfeed, leveller, and clipping behavior.

What it **cannot** verify with this rig:

- **Inter-slot alignment** between the different S/PDIF slots / I2S / PDM (the firmware's
  full alignment guarantee) — needs a multi-channel digital capture or a second receiver.
- **I2S and PDM outputs** — the USBrx only receives S/PDIF.
- **Sample-rate sweeps** — the suite runs at 48 kHz by design.

---

## 15. How it stays safe

- The whole audio group is **opt-in** (`--audio`) and **skips** cleanly when the rig/deps
  are absent, so it never breaks a normal control-plane run.
- It writes **no flash** — every change is to live RAM state.
- Each test restores what it changed, and the runner takes a **full snapshot before the
  run and restores it after**, confirming "live state restored byte-for-byte".
- The test signals are digital, captured by the USBrx; nothing is played on speakers.

---

## 16. File map

| File | Role |
|---|---|
| `tools/dspi_test/run.py` | CLI entry point and runner (`python3 -m tools.dspi_test.run`). |
| `tools/dspi_test/framework.py` | Test registry, the `Check` assertions, the serial runner, report output. |
| `tools/dspi_test/device.py` | USB vendor-command transport to the DSPi (`pyusb`). |
| `tools/dspi_test/profile.py` | Detects the attached board's capabilities (channels, slots, etc.). |
| `tools/dspi_test/lifecycle.py` | Pre-run snapshot and post-run restore. |
| `tools/dspi_test/audio.py` | The audio measurement engine (device discovery, play/record, sweeps, metrics) + the `--list`/`--probe` tool. |
| `tools/dspi_test/tests/audio_loopback.py` | The `audio` group: all 40 loopback tests + their fixtures and parameters. |
| `tools/dspi_test/tests/*.py` | The control-plane test modules (eq, outputs, inputs, volume, presets, etc.). |
| `tools/filter_tester/compare_filter.py` | RBJ filter reference reused for the expected PEQ responses. |
| `tools/dspi_test/README.md` | The short quick-start (this guide is the long version). |

---

*Related repos:* [DSPi](https://github.com/WeebLabs/DSPi) ·
[USBrx](https://github.com/WeebLabs/USBrx) ·
[pico_spdif_rx](https://github.com/elehobica/pico_spdif_rx)
