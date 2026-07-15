# Audio Code Quality Review — ADAT Input Feature (release/v1.1.5)

**Date**: 2026-07-13
**Reviewer**: Senior QA / DSP engineer (studio-grade audit)
**Platform**: RP2350 (feature); RP2040 (state round-trip only)

**Files Reviewed**:
- NEW: `firmware/DSPi/adat_input.c`, `adat_input.h`, `adat_input.pio`, `input_servo.c`, `input_servo.h`, `Documentation/Features/adat_input_spec.md`, `tools/adat_rx_test/adat_rx_roundtrip.c`
- MODIFIED: `main.c`, `vendor_commands.c`, `bulk_params.c/.h`, `flash_storage.c`, `audio_input.c/.h`, `audio_pipeline.c`, `adat_output.c`, `spdif_input.c`, `notify.c/.h`, `config.h`, `usb_audio.c`, `CMakeLists.txt`, `scripts/check_ram_placement.py`

**Severity Summary**: 1 Critical | 0 Major | 3 Minor | 7 Informational/Notes

## Executive Summary

This is high-quality, disciplined firmware. The PIO NRZI decoder is timing-correct
to the cycle, the frame decode math is bit-exact (proven by the round-trip harness
at all 32 bit offsets), the servo extraction is behaviorally identical to its
SPDIF origin, and the persistence/version-migration work is complete and careful.
Both platforms build clean, the RAM budget check passes, and the hot decode path
is verified RAM-resident. There is exactly one real defect: the standalone
sample-rate-change handler does not defer the output restart for the ADAT source
the way it does for I2S, so `complete_pipeline_reset()` un-mutes the outputs before
the receiver has re-locked and re-prefilled. That produces an audible click on
every ADAT-slave upstream rate change and a persistent un-mute in the master-mode
96 kHz "park" case. Inter-slot sample alignment itself is NOT at risk anywhere in
this diff.

## Verification performed

- Round-trip decoder test (`adat_rx_roundtrip.c`): **PASS** — all 32 bit offsets, 64 frames, samples bit-exact.
- RP2350 build: **clean**. RP2040 build: **clean**.
- `check_ram_placement.py` on both ELFs: **PASS** (RP2350 `.data` 67952 < 73728 budget; hot-path Check B 0 FAIL — decode path is RAM-resident and does not branch to flash in steady state).
- PIO instruction timing verified against the generated header `build-rp2350/DSPi/adat_input.pio.h` (see INFO-1).
- `input_servo_apply()` diffed line-for-line against the removed `spdif_input.c` servo body (see INFO-2).

---

## Critical Issues

### CRIT-001: ADAT rate change un-mutes outputs before re-lock (missing `defer` for ADAT in the standalone rate-change handler)

- **File**: `firmware/DSPi/main.c:2338` (and the omission at `main.c:1101`)
- **Category**: Real-Time Safety / Signal Integrity (mute-until-locked contract)
- **Description**: The deferred rate-change handler is:
  ```c
  perform_rate_change(r, active_input_source == INPUT_SOURCE_I2S);   // line 2338
  ```
  The second argument is `defer_output_to_input_prefill`. Inside
  `perform_rate_change()` the output restart is:
  ```c
  if (!spdif_prefilling && !defer_output_to_input_prefill) complete_pipeline_reset();
  ```
  For an ADAT source, `defer` evaluates to **false** and `spdif_prefilling` is
  false (ADAT uses its own `adat_prefilling` flag), so `complete_pipeline_reset()`
  runs. That function re-enables all output slots (Phase 2) and, critically, its
  Phase 4 mute-release exception at `main.c:1101` lists only
  `INPUT_SOURCE_I2S` and `INPUT_SOURCE_SPDIF` — **not** `INPUT_SOURCE_ADAT` — so
  `dac_hw_mute_release()` fires. The outputs are therefore enabled and un-muted
  while the ADAT receiver is still RELOCKING/SYNCING (or parked), i.e. before the
  ADAT main-loop block has drained, prefilled, and re-enabled in sync. The ADAT
  block deliberately does NOT set `adat_prefilling` when a rate change is pending
  (it takes the `if (!adat_input_check_rate_change())` else-path at `main.c:1848`),
  so neither the `spdif_prefilling` skip nor the `defer` skip protects it. This is
  a direct asymmetry with the I2S precedent, which passes `defer = (active == I2S)`
  and is fully protected.
- **Impact**:
  - **Slave mode, upstream rate change (common):** wire rate changes -> ADAT drops
    lock -> re-locks at new rate -> `check_rate_change()` arms the pipeline change
    -> this handler runs `complete_pipeline_reset()` which un-mutes against
    freshly-drained (empty) consumer pools. Audible click/underrun, then the ADAT
    block immediately re-drains and re-prefills. A click on every source-side rate
    change.
  - **Master mode, `REQ_SET_INPUT_RATE`=96000 (the documented "park"):**
    `adat_input_on_rate_change()` parks (state ACQUIRING, `rate_ok=false`), then
    `complete_pipeline_reset()` un-mutes and enables the outputs. Because ADAT
    never reaches LOCKED while parked, the re-prefill gate never fires, so the DAC
    stays **un-muted indefinitely** feeding an underrunning pipeline — a direct
    violation of the spec's "outputs muted while parked" guarantee
    (adat_input_spec.md line 52).
  - Note: inter-slot ALIGNMENT is preserved (every start is a synchronized start);
    this is an audible-artifact / mute-contract defect, not a slot-drift defect.
- **Evidence**: `main.c:2338` (defer excludes ADAT); `main.c:1101` (Phase 4
  exception excludes ADAT); ADAT block else-path at `main.c:1846-1854` never sets
  `adat_prefilling` on the rate-change branch.
- **Recommendation**: Mirror the I2S precedent at line 2338:
  ```c
  perform_rate_change(r, active_input_source == INPUT_SOURCE_I2S ||
                         active_input_source == INPUT_SOURCE_ADAT);
  ```
  With `defer=true`, `complete_pipeline_reset()` is skipped, the mute is retained,
  and the ADAT main-loop block owns the drain/prefill/enable + mute release exactly
  as it does on first lock. (Adding `INPUT_SOURCE_ADAT` to the Phase 4 exception at
  line 1101 would also be defensively correct, but fixing the defer flag alone is
  sufficient and matches the I2S design.) Verify the master 96 kHz park case stays
  muted after the fix.

---

## Major Issues

None.

---

## Minor Issues

### MIN-001: RP2040 `REQ_GET_ADAT_INPUT_STATUS` comment/spec says "16 zero bytes"; code sends 20

- **File**: `firmware/DSPi/vendor_commands.c` (`case REQ_GET_ADAT_INPUT_STATUS`, comment `// ADAT unavailable: 16 zero bytes`); `Documentation/Features/adat_input_spec.md:136`
- **Category**: Code Quality / Documentation accuracy
- **Description**: `AdatInputStatusPacket` is 20 bytes packed (8×u8 + u16 + u16 + u32 + u32). The RP2040 path does `memset(&s,0,sizeof(s)); vendor_send_response(&s,sizeof(s));`, i.e. it correctly sends **20** zero bytes. Two comments (the vendor case and the spec's "On RP2040" bullet) claim 16. No functional impact — a host that parses the 20-byte struct gets the right length — but the doc/comment is wrong and could mislead a host implementer.
- **Recommendation**: Change both "16" references to "20".

### MIN-002: Bulk/flash ADAT disable-refusal does not consider a pending source switch to ADAT (vendor does)

- **File**: `firmware/DSPi/bulk_params.c` (`!want && adat_input_enabled` branch) and `firmware/DSPi/flash_storage.c` `io_config_apply()` (`en==0 && active==ADAT` branch)
- **Category**: Robustness / consistency
- **Description**: `REQ_SET_ADAT_INPUT_ENABLE` refuses a disable when ADAT is active OR is the target of a pending source switch (`is_pending`). The bulk-apply and flash-apply disable paths check only `active_input_source == INPUT_SOURCE_ADAT`, not the pending case. A bulk/preset that both disables ADAT and (elsewhere) selects it could disable it while a switch to ADAT is armed; the switch then silently no-ops via `input_source_selectable()`. No crash and no misalignment, but the three "same rule" paths are not identical.
- **Recommendation**: For parity, also treat `input_source_change_pending && pending_input_source == INPUT_SOURCE_ADAT` as "keep enabled" in the bulk/flash disable branches, or explicitly document the intended difference.

### MIN-003: Redundant double receiver restart when a clock-mode flip and a pin/enable restart are armed together

- **File**: `firmware/DSPi/main.c` — deferred clock-mode handler (~2989) runs before `adat_input_restart_pending` handler (~3405)
- **Category**: Efficiency (benign)
- **Description**: If both `adat_clock_mode_change_pending` and `adat_input_restart_pending` are set in the same iteration (e.g. a bulk push that changes both pin and clock mode while ADAT is live), the clock-mode handler stops/starts the receiver, then the restart handler stops/starts it again. Both run under mute, so there is no correctness or alignment problem — just a wasted mute+relock cycle.
- **Recommendation**: Optional: clear `adat_input_restart_pending` inside the live clock-mode-flip branch (it already fully restarts the receiver), or accept the redundancy as harmless.

---

## Informational / Notes

### INFO-1 (PIO timing): CLEAN — verified instruction-by-instruction
Against `build-rp2350/DSPi/adat_input.pio.h`: every path emits exactly one bit per
8-cycle cell. Low half (addr 1..4): poll@1 -> [3] -> poll@3 -> emit0 [3] wrap = 8
cycles, polls 4 apart. High half (addr 6,8,9,11,12): poll@6 -> [3] -> poll@9 ->
emit0 [2]+jmp = 8 cycles, polls 4 apart. Rise (poll@1/@3 -> h_one `in y[6]`=7c ->
poll@6) = 8 cycles to the opposite half's first poll. Fall (poll@6/@9 -> jmp,jmp ->
l_one `in y[5]`=6c -> poll@1) = 8 cycles. The 7-vs-6 asymmetry exactly compensates
the 1-cycle (rise, single jmp) vs 2-cycle (fall, jmp+jmp) entry costs. Setup in
`adat_input_start` is correct: shift-left/MSB-first, autopush 32, `jmp_pin`=RX GPIO,
initial PC = `wrap_target` (l_p0), `y`=1 seed, divider = round(sys*256/(2048*fs)) =
16.8 form of sys/(2048·Fs) -> PIO clock 2048·Fs.

### INFO-2 (servo refactor): CLEAN — behaviorally identical
`input_servo_apply()` is a verbatim lift of the removed `spdif_input.c` servo body.
Rate sanity clamp (20k..200k), `sys_clk/actual`, fill trim (±2 deadband, `KP`
0.0005, /16 scale), `i2s_div = spdif_div*2` lock, unchanged-divider skip, the
per-slot SPDIF/I2S write loop, `adat_output_servo_divider(spdif_div)` under
`#if PICO_RP2350`, and the CLK_GPOUTn MCK 24.8 math are all preserved. The only
deltas are (a) dropping the previously-dead `i2s_div_f` local and (b) returning
`spdif_div` instead of void. `spdif_input_current_tx_divider()` semantics are
preserved (LOCKED-gated `input_servo_current_divider()` == old LOCKED-gated
`last_spdif_div`). The now-shared divider cache is safe: every servoed-lock path
(`spdif_input_start`, ADAT SYNCING->LOCKED) calls `input_servo_reset()` before the
first apply, so a stale cross-source cached divider can never suppress a needed
write, and `restore_nominal_spdif_dividers()` on source-switch-away is always
followed by a reset on the next lock.

### INFO-3 (decoder math): CLEAN — bit-exact and proven
`adat_rx_header_ok` (`(v>>(52-k))&0xFFF == 0x801`) and `adat_rx_decode_frame`
(k==0 fast path; k>0 spill via `(prev<<k)|(next>>(32-k))`; 30-bit field extract via
`(v64>>(34-sh))&0x3FFFFFFF`; nibble unstuff of positions 25/20/15/10/5/0; s24<<8
sign extension) are the exact inverse of `adat_encode_frame`, confirmed by the
round-trip harness at all 32 offsets including corner samples. The
`frames=(avail-1)/8` guard correctly reserves the k>0 spill word; `avail` and every
ring index are masked; `widx` is sampled once per poll and the CPU reads strictly
behind it, so no torn 32-bit word can be read. The whole-frame lap guard (ring is
256 whole frames) preserves `frame_bit` phase. The 192-frame cap matches the
`buf_l/buf_r/buf_in_ext[..][192]` capacity exactly — no overrun.

### INFO-4 (alignment — the inviolable constraint): CLEAN
No new path can misalign output slots. Only one input source is ever active, so
only one servo runs, and `input_servo_apply()` writes an identical `spdif_div` to
every SPDIF slot, `2×spdif_div` to every I2S slot, and the same `spdif_div` to the
ADAT output — the exact-2× lock prevents rounding drift. Every ADAT lock/loss/
rate/mode/pin/boot transition re-establishes alignment through the shared
`prepare_pipeline_reset` / `drain_and_disable_outputs` / `enable_outputs_in_sync` /
`complete_pipeline_reset` primitives, each of which starts all slots in a single
synchronized PIO start. CRIT-001 un-mutes early but still starts all slots
together, so it is an audible-artifact defect, not an alignment defect.

### INFO-5 (concurrency / RT safety): CLEAN
`adat_rx_state` is volatile and only touched from the main-loop poll; the other
state is deliberately main-loop-only and never read/written from an ISR (the RX
DMA has no IRQ). Cross-context deferred flags (`adat_clock_mode_change_pending`,
`pending_adat_clock_mode`, `adat_input_restart_pending`) are volatile with `__dmb`
barriers, matching the I2S precedent. `notify_push_adat_input_state` brackets the
ring push with `save_and_disable_interrupts`. `DSP_TIME_CRITICAL` placement matches
the spec (poll / rate machine / decode / servo in RAM; `adat_rx_scan` intentionally
in flash, reached only while muted). `printf` appears only in start/stop. The rate
machine (RAM) calls flash helpers (`snap_rate`, `drop_lock`, `set_divider`,
`set_state`) only on 32 ms window boundaries or state transitions, not in the
per-poll steady state; Check B confirms no hot-path flash branch.

### INFO-6 (boot-into-ADAT branch): defensive-only, safe
`active_input_source` is only ever assigned ADAT via the `input_source_selectable`-
gated main-loop switch handler (`main.c:3272`); `apply_slot_to_live` merely arms a
pending switch. So the boot block's `else if (active_input_source==INPUT_SOURCE_ADAT)`
(main.c:1652) is defensive parity with the I2S branch and is not the normal path.
Even if reached with an unset pin, `adat_input_start()` no-ops (prints, leaves
`adat_rx_running=false`), so the worst case is "no audio, muted", not a fault.
`input_source_valid()` now accepting 3 on both platforms is safe because the flash
apply path (`flash_storage.c:2888`) is re-gated by `input_source_selectable()` in
the main-loop handler (false on RP2040 / when disabled / when pin-less).

### INFO-7 (persistence / vendor consistency): CLEAN
WIRE_FORMAT_VERSION 24 (3 fields claimed from `WireInputConfig` reserved bytes,
struct still 16 B, +1/absent conventions correct), SLOT_DATA_VERSION 32 (tail
append; `SLOT_DATA_SIZE_V31` correctly re-based to end at `adat_input_pin`; switch
table updated), directory V15 with a complete `FlashOutputConfig_v14` /
`PresetDirectory_v14` freeze and a CRC-validated V14->V15 migration that seeds
`adat_input_pin=0xFF` (all older migration arms also seed it, avoiding the
"zero-fill reads as GPIO 0" trap). `io_config_from_slot` gates the slot read on
`version>=32`. The three pin-validation sites (`REQ_SET_*`, bulk, flash) share the
ADAT-TX-pin loopback exception, the enable-needs-valid-pin rule, and self-block
avoidance (new pin always differs from the current, so `is_pin_in_use` cannot fire
on itself); `pin_used_by_fixed_peripheral` claims the RX pin only while enabled.
Both platforms build; no em-dashes in any new text (project style rule upheld).

---

## Positive Observations

- The NRZI receiver design (edge re-anchoring PIO + free-running ENDLESS ring with
  no IRQ + CPU frame sync) is elegant and genuinely lower-overhead than an
  IRQ-driven approach; the "dark line decodes as zeros, header verify is the loss
  detector" insight removes a whole class of timeout logic.
- Shipping a host-side round-trip harness that exercises the real encoder against
  the real decoder at all 32 bit offsets is exactly the right way to lock down
  bit-exactness; it passes and is trivially re-runnable.
- The servo extraction into `input_servo.c` is a clean, verbatim, well-documented
  refactor that removes duplication without changing behavior.
- Version migration coverage is exhaustive and defensive (every historical
  directory arm seeds the new field correctly).

## Recommendations Summary (prioritized)

1. **Fix CRIT-001**: add `|| active_input_source == INPUT_SOURCE_ADAT` to the defer
   argument at `main.c:2338` (and, defensively, to the Phase 4 mute-release
   exception at `main.c:1101`). Re-test: ADAT-slave upstream rate change (expect no
   click) and ADAT-master `REQ_SET_INPUT_RATE`=96000 (expect outputs stay muted).
2. Correct the "16 zero bytes" -> "20" in the vendor comment and spec (MIN-001).
3. Align the bulk/flash disable-refusal with the vendor `is_pending` check, or
   document the difference (MIN-002).
4. Optionally suppress the redundant clock-mode + restart double-restart (MIN-003).

## Verdict

**Ship after fixing CRIT-001.** The receiver, decoder, servo, and persistence work
are studio-grade and verified (bit-exact round trip, clean builds, RAM budget and
placement pass, alignment invariant upheld). The single critical defect is a
one-line divergence from the already-correct I2S rate-change flow: `perform_rate_change`
is not told to defer the output restart for ADAT, so the DAC un-mutes before the
receiver re-locks — audible on every source-clock rate change and a persistent
un-mute in the 96 kHz park case. It does not threaten inter-slot sample alignment,
but it violates the mute-until-locked contract and must be fixed before release.
The remaining items are cosmetic or defensive.

---

## Resolution (post-audit, 2026-07-13)

- CRIT-001: FIXED. The deferred rate-change handler (main.c) now defers the
  output restart to the input prefill block for ADAT as well as I2S, so
  complete_pipeline_reset() cannot un-mute before the receiver re-locks.
- MIN-001: FIXED. All "16 zero bytes" / "16-byte" references corrected to the
  actual 20-byte AdatInputStatusPacket (vendor comment, spec, architecture doc,
  adat_input.h).
- MIN-002: FIXED. Bulk and flash disable-refusal now also check a pending
  source switch targeting ADAT, matching the vendor path.
- MIN-003: ACCEPTED AS-IS. The double restart on a simultaneous clock-mode
  flip + pin change is muted and harmless; serializing the two handlers would
  add ordering complexity for no audible benefit.

---

## Post-release timing defect and redesign (2026-07-13, second addendum)

An external review reported that the RX PIO divider rounding could not be
absorbed by transition re-anchoring, predicting bit deletion at 44.1 kHz. A
cycle-accurate simulation of the generated program including the fractional
clock divider (tools/adat_rx_test/adat_rx_bitdiff.c) CONFIRMED the defect
and showed it to be worse than reported: the original decoder's poll
schedule (next poll one full cell after each detected edge) is a one-way
ratchet, so any interval where the decoder runs locally slower than the
wire converts to permanent phase debt repaid only by deleting bits. The
fractional divider guarantees locally slow cells at 44.1 kHz (27/28-cycle
cell modulation), so 44.1 kHz could never lock at ANY nearby divider, and
both proposed mitigations were insufficient: fast-biasing the divider
(empirical law: clean only when the stream bit is at least the longest
INSTANTANEOUS cell, not the mean) and an added early poll (defeated by the
same cell-length modulation).

Fix: the decoder was redesigned to run at clock divider 1.0 (no fractional
jitter at all) counting each cell with a 2-cycle poll loop (length per rate
via the Y register: 27/25 sys cycles), with a post-edge window placed so
edges re-anchor the grid within 2 sys cycles in both directions. The
cycle-accurate model proves the new program bit-exact at 44.1 and 48 kHz
across at least +-1000 ppm of source clock offset and that the original
program's failures still reproduce (model fidelity check). The 150 MHz
fallback sys clock is documented as unsupported for ADAT input.

Follow-up, 2026-07-15: real external hardware showed that using the corrupt
cross-rate decoded stream's DMA fill rate as the family-rate bootstrap was
not robust at 44.1 kHz. Slave acquisition now probes the exact 48 and 44.1
kHz cell timings in turn and accepts only an eight-header valid run; DMA-rate
measurement begins after that structural proof and is retained for fine servo
tracking only. The regression now verifies wrong-candidate rejection followed
by matching-candidate acceptance in both directions.
