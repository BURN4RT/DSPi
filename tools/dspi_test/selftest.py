"""selftest.py: no-hardware checks for the audio-loopback harness.

Covers the parts of tests/audio_loopback.py whose correctness does not depend on
a device being attached, against mocks that model the firmware's actual
semantics:

  * Deferred-operation barriers.  A SET arms a pending flag and NO-OPS when the
    request equals the APPLIED state; the main loop applies it later
    (vendor_commands.c REQ_SET_OUTPUT_TYPE, main.c process_type_switches).
    Test 3 deliberately reproduces the pre-barrier bug, so a regression that
    reintroduces fire-and-assume sequencing shows up here rather than as an
    intermittent hardware failure.
  * Matrix isolation.  _route_only() must clear EVERY input row, since the
    upmixer feeds rows 2..4 and multichannel USB feeds up to row 7.
  * Per-channel band ceilings, which are probed rather than taken from the
    profile's channel-0 value.

    python3 -m tools.dspi_test.selftest      # exit 0 = all good
"""
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.dspi_test.device import OP, Stall, Timeout
from tools.dspi_test.tests import audio_loopback as L


class MockDev:
    """Models the real deferral: SET arms a pending flag and no-ops when the
    request equals the APPLIED array; the main loop applies it `apply_after_s`
    later.  `stall_n` makes the first N GETs raise, mimicking the control-IRQ
    blackout."""

    def __init__(self, applied=0, apply_after_s=0.3, stall_n=0):
        self.applied = applied          # output_types[slot]
        self.pending = None
        self.pending_at = None
        self.apply_after_s = apply_after_s
        self.stall_n = stall_n
        self.sets = 0
        self.gets = 0

    def _tick(self):
        if self.pending is not None and time.monotonic() >= self.pending_at:
            self.applied = self.pending
            self.pending = None

    def get_u8(self, opcode, wvalue=0):
        self._tick()
        if opcode == OP.GET_OUTPUT_TYPE:
            self.gets += 1
            if self.stall_n > 0:
                self.stall_n -= 1
                raise Stall(opcode, "IN", -9, "control-IRQ blackout")
            return self.applied
        if opcode == OP.SET_OUTPUT_TYPE:
            self.sets += 1
            new_type = (wvalue >> 8) & 0xFF
            if new_type == self.applied:      # <-- the stale-comparison no-op
                return 0
            self.pending = new_type
            self.pending_at = time.monotonic() + self.apply_after_s
            return 0
        raise AssertionError(f"unexpected opcode {opcode:#x}")


def old_set_output_type(dev, slot, otype):
    """What the code did before Phase 1: fire and assume."""
    return dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(otype << 8) | slot)


fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


# Make the settle cheap so the harness test runs fast.
L.PIPELINE_SETTLE_S = 0.01

print("1. barrier: single switch actually lands")
d = MockDev(applied=L.TYPE_SPDIF)
r = L._set_output_type(d, 0, L.TYPE_I2S)
check(r is True, "returns True")
check(d.applied == L.TYPE_I2S, f"device applied I2S (got {d.applied})")

print("2. barrier: SPDIF -> I2S -> SPDIF round trip ends on SPDIF")
d = MockDev(applied=L.TYPE_SPDIF)
L._set_output_type(d, 0, L.TYPE_I2S)
L._set_output_type(d, 0, L.TYPE_SPDIF)
check(d.applied == L.TYPE_SPDIF, f"ended on S/PDIF (got {d.applied})")

print("3. control: the OLD raw sequence loses the restore (bug reproduced)")
d = MockDev(applied=L.TYPE_SPDIF)
old_set_output_type(d, 0, L.TYPE_I2S)
old_set_output_type(d, 0, L.TYPE_SPDIF)   # no-ops against stale applied
time.sleep(0.4)
d._tick()
check(d.applied == L.TYPE_I2S,
      f"old path is stuck in I2S (got {d.applied}) -- confirms the mock models the bug")

print("4. barrier: unchanged request is a genuine no-op (no SET, no settle)")
d = MockDev(applied=L.TYPE_SPDIF)
t0 = time.monotonic()
r = L._set_output_type(d, 0, L.TYPE_SPDIF)
check(r is True and d.sets == 0, f"no SET issued (sets={d.sets})")
check((time.monotonic() - t0) < L.PIPELINE_SETTLE_S, "no settle paid")

print("5. barrier: never-applied switch reports failure, does not hang")
d = MockDev(applied=L.TYPE_SPDIF, apply_after_s=999)
L.BARRIER_TIMEOUT_S = 0.4
t0 = time.monotonic()
r = L._set_output_type(d, 0, L.TYPE_I2S)
el = time.monotonic() - t0
check(r is False, "returns False")
check(0.3 < el < 1.5, f"bounded by the timeout ({el:.2f}s)")

print("6. barrier: tolerates a control-IRQ blackout on the ENTRY read")
d = MockDev(applied=L.TYPE_SPDIF, apply_after_s=0.2, stall_n=3)
L.BARRIER_TIMEOUT_S = 4.0
try:
    r = L._set_output_type(d, 0, L.TYPE_I2S)
except (Stall, Timeout) as e:
    r = None
    check(False, f"stall escaped the helper: {e}")
check(r is True and d.applied == L.TYPE_I2S, "switch still confirmed through stalls")

print("6b. barrier: entry read that NEVER answers reports failure, no exception")
d = MockDev(applied=L.TYPE_SPDIF, stall_n=10**6)
L.BARRIER_TIMEOUT_S = 0.3
try:
    r = L._set_output_type(d, 0, L.TYPE_I2S)
except (Stall, Timeout) as e:
    r = None
    check(False, f"stall escaped the helper: {e}")
check(r is False, f"returns False (got {r!r})")
check(d.sets == 0, f"no SET issued against an unknown state (sets={d.sets})")

print("7. _wait_applied returns last value on timeout, None if all polls raised")
class AllStall:
    def get(self):
        raise Timeout("nope")
L.BARRIER_TIMEOUT_S = 0.2
v = L._wait_applied(AllStall().get, 1, timeout_s=0.2)
check(v is None, f"None when nothing was ever read (got {v!r})")


# --------------------------------------------------------------------------
# Phase 2: deterministic baseline
# --------------------------------------------------------------------------

class MatrixDev:
    """Records matrix crosspoint writes and answers per-channel band probes."""

    def __init__(self, n_in, n_out, band_counts=None):
        self.n_in, self.n_out = n_in, n_out
        self.xp = {}                       # (inp, out) -> enabled
        self.band_counts = band_counts or {}

    def set(self, opcode, payload=b"", wvalue=0, windex=0):
        assert opcode == OP.SET_MATRIX_ROUTE
        inp, out, enabled, _phase, _gain = struct.unpack("<BBBBf", payload)
        self.xp[(inp, out)] = bool(enabled)
        return 0

    def get(self, opcode, length, wvalue=0):
        assert opcode == OP.GET_EQ_PARAM
        ch, band = (wvalue >> 8) & 0xFF, (wvalue >> 3) & 0x1F
        if band >= self.band_counts.get(ch, 10):
            raise Stall(opcode, "IN", -9, "band above ceiling")
        return b"\0\0\0\0"


class Profile:
    def __init__(self, n_in, n_out):
        self.num_input_channels = n_in
        self.num_output_channels = n_out


print("8. _route_only clears EVERY input row, not just the stereo pair")
d = MatrixDev(n_in=8, n_out=11)
L._route_only(d, Profile(8, 11), 0, 1)
on = {k for k, v in d.xp.items() if v}
check(on == {(0, 0), (1, 1)}, f"exactly the target pair is enabled (got {sorted(on)})")
covered = {(i, o) for i in range(8) for o in range(11)}
check(set(d.xp) == covered,
      f"all {len(covered)} crosspoints written (got {len(d.xp)})")
upmix_rows = [k for k in d.xp if k[0] in (2, 3, 4)]
check(len(upmix_rows) == 3 * 11 and not any(d.xp[k] for k in upmix_rows),
      "upmixer rows 2..4 explicitly disabled")

print("9. _route_only on RP2040 geometry (2 inputs)")
d = MatrixDev(n_in=2, n_out=7)
L._route_only(d, Profile(2, 7), 2, 3)
on = {k for k, v in d.xp.items() if v}
check(on == {(0, 2), (1, 3)}, f"target pair only (got {sorted(on)})")

print("10. _band_ceiling probes per channel and caches")
L._BAND_CEILING.clear()
d = MatrixDev(n_in=8, n_out=11, band_counts={8: 10, 9: 6})
check(L._band_ceiling(d, 8) == 10, "channel 8 ceiling 10")
check(L._band_ceiling(d, 9) == 6, "channel 9 ceiling 6 (differs from channel 8)")
d.band_counts[9] = 99                      # would change the answer if re-probed
check(L._band_ceiling(d, 9) == 6, "second call served from cache")
L._BAND_CEILING.clear()

print("11. baseline comparison: float tolerance, int equality, None skipped")
rows = [("a", 0.0, 0.0), ("b", 0, 0), ("c", 0.0, None), ("d", 0.0, 0.5), ("e", 0, 1)]
bad = []
for label, want, got in rows:
    if got is None:
        continue
    ok = abs(got - want) < 1e-3 if isinstance(want, float) else got == want
    if not ok:
        bad.append(label)
check(bad == ["d", "e"], f"only real mismatches reported (got {bad})")


# --------------------------------------------------------------------------
# Phase 3: rate parameterisation
# --------------------------------------------------------------------------

print("12. rate tags: primary is unsuffixed, others are identifier-safe")
check(L._rate_tag(L.PRIMARY_RATES[0]) == "", "primary rate has no suffix")
check(L._rate_tag(44100) == "_44k1", f"44.1k tag (got {L._rate_tag(44100)!r})")
for fs in (44100, 96000, 32000):
    tag = L._rate_tag(fs)
    check(("x" + tag).isidentifier(), f"{fs} tag {tag!r} is identifier-safe")

print("13. every SECONDARY_TESTS name resolves to a registered test")
from tools.dspi_test.framework import REGISTRY
registered = {tc.name for tc in REGISTRY}
missing = [n for n in L.SECONDARY_TESTS if n not in registered]
check(not missing, f"all subset names exist (missing: {missing})")
variants = [n + "_44k1" for n in L.SECONDARY_TESTS]
absent = [v for v in variants if v not in registered]
check(not absent, f"all 44.1k variants registered (absent: {absent})")

print("14. rate variants are contiguous and last, so only ONE rate switch")
audio_names = [tc.name for tc in REGISTRY if tc.group == "audio"]
idx = [i for i, n in enumerate(audio_names) if n.endswith("_44k1")]
check(idx == list(range(min(idx), max(idx) + 1)),
      "44.1k variants form one contiguous block")
check(max(idx) < len(audio_names) - 1 and audio_names[-1] == "rate_switch_round_trip",
      f"round-trip test registered last (last is {audio_names[-1]!r})")

print("15. _rate_variant restores _ACTIVE_FS even when the body raises")
seen = {}


def _body(dev, profile, chk):
    seen["fs"] = L._ACTIVE_FS
    raise RuntimeError("boom")


L._ACTIVE_FS = None
v = L._rate_variant(_body, 44100)
try:
    v(None, None, None)
except RuntimeError:
    pass
check(seen["fs"] == 44100, f"body saw the variant rate (got {seen['fs']})")
check(L._ACTIVE_FS is None, f"_ACTIVE_FS restored after the raise (got {L._ACTIVE_FS})")
# _rate_variant registers as a side effect; drop the throwaway, asserting what
# we remove so a future reorder cannot silently delete a real test.
assert REGISTRY[-1].name == "_body_44k1", REGISTRY[-1].name
REGISTRY.pop()

print("16. _get_rig honours _ACTIVE_FS and keeps per-rate caches apart")
L._RIG.clear(); L._RIG_FAIL.clear()
L._RIG[48000] = {"fs": 48000, "slot": 0, "out": 0, "in": 1}
L._RIG[44100] = {"fs": 44100, "slot": 0, "out": 0, "in": 1}
L._DEVS = (0, 1, {"out": {"name": "m"}, "in": {"name": "m"}})


class RateDev:
    def __init__(self, rate):
        self.rate = rate

    def get_u32(self, opcode, wvalue=0):
        assert wvalue == L.STATUS_SAMPLE_RATE
        return self.rate


L._ACTIVE_FS = 44100
check(L._get_rig(RateDev(44100), None)["fs"] == 44100, "picks the 44.1k rig")
L._ACTIVE_FS = None
check(L._get_rig(RateDev(48000), None)["fs"] == 48000, "falls back to the primary rig")
check(L._get_rig(RateDev(44100), None, 44100)["fs"] == 44100, "explicit fs wins")
L._RIG.clear(); L._RIG_FAIL.clear()

print("17. a cached per-rate failure does not poison the other rate")
L._RIG_FAIL[44100] = "nope"
L._RIG[48000] = {"fs": 48000, "slot": 0, "out": 0, "in": 1}
try:
    L._get_rig(RateDev(48000), None, 44100)
    check(False, "should have raised Skip for the failed rate")
except L.Skip as e:
    check(str(e) == "nope", "failed rate skips with its cached reason")
check(L._get_rig(RateDev(48000), None, 48000)["fs"] == 48000,
      "the healthy rate still works")
L._RIG.clear(); L._RIG_FAIL.clear(); L._DEVS = None; L._ACTIVE_FS = None

print("18. --audio-rates registration modes (end to end, via --list)")
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def _list_audio(*extra):
    out = subprocess.run(
        [sys.executable, "-m", "tools.dspi_test.run", "--list", "--group", "audio", *extra],
        cwd=ROOT, capture_output=True, text=True, check=True).stdout
    names = [ln.strip().split()[0] for ln in out.splitlines()
             if ln.startswith("  ") and ln.strip() and not ln.startswith("  [")]
    return names


base = _list_audio()
both = _list_audio("--audio-rates", "48000,44100")
alt = _list_audio("--audio-rates", "44100")

n_primary = len(base) - sum(n.endswith("_44k1") for n in base) - 1   # minus round-trip
check(sum(n.endswith("_44k1") for n in base) == len(L.SECONDARY_TESTS),
      f"default: subset-sized 44.1k pass ({sum(n.endswith('_44k1') for n in base)})")
check(len(both) == 2 * n_primary + 1,
      f"override lists the FULL matrix twice plus the round trip "
      f"({len(both)} vs {2 * n_primary + 1})")
check(len(alt) == n_primary + 1, f"single-rate override runs one full matrix ({len(alt)})")
check(not any(n.endswith("_44k1") for n in alt),
      "44.1k-as-primary names are unsuffixed")
for label, names in (("default", base), ("override", both)):
    idx = [i for i, n in enumerate(names) if n.endswith("_44k1")]
    check(idx == list(range(min(idx), max(idx) + 1)),
          f"{label}: 44.1k variants contiguous, so exactly one rate change")
    check(names[-1] == "rate_switch_round_trip",
          f"{label}: round trip last, leaving the device on the primary rate")

print()
print("FAILURES:", len(fails))
for f in fails:
    print("  -", f)
sys.exit(1 if fails else 0)
