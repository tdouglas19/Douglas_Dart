"""Suite-wide defaults.

The legacy tests exercise PULSEJET_MODE at many (mach, altitude) points and
were written against the native/km dispatch timing. With pulsejet-fp as the
guarded primary (2026-08-12), each such point would run a real ~30-90 s
transient sim, turning the suite from minutes into hours -- so the suite
runs with the FP primary disabled BY DEFAULT via the documented
kill-switch. The pulsejet-fp integration itself is tested explicitly in
test_pulsejet_fp_bridge.py / test_pulsejet_fp_bridge_live.py, which
re-enable the switch per-test. Production runs (pipeline, optimizer,
trajectory tools) do not set this variable and therefore get the
first-principles primary.
"""
import os

os.environ.setdefault("DOUGLAS_DART_DISABLE_PULSEJET_FP", "1")
