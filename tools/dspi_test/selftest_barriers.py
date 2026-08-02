"""selftest_barriers.py — no-hardware checks for the audio-loopback harness.

Exercises the deferred-operation barriers in tests/audio_loopback.py against a
mock that models the firmware's actual semantics: a SET arms a pending flag and
NO-OPS when the request equals the APPLIED state, and the main loop applies it
later (vendor_commands.c REQ_SET_OUTPUT_TYPE, main.c process_type_switches).
Test 3 deliberately reproduces the pre-barrier bug, so a regression that
reintroduces fire-and-assume sequencing shows up here rather than as an
intermittent hardware failure.

    python3 -m tools.dspi_test.selftest_barriers      # exit 0 = all good
"""
import sys, time
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

print()
print("FAILURES:", len(fails))
for f in fails:
    print("  -", f)
sys.exit(1 if fails else 0)
