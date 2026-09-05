"""Regression tests.

The two reference logs used here were analysed by hand before this toolkit existed.
These tests pin the toolkit against the numbers those analyses produced - if a refactor
changes them, the refactor is wrong until proven otherwise.

The logs are not distributed with the repo (they are ~14 MB of someone's flight data).
Set LOG_DIR to a directory containing them, or just run the tests without: everything
that needs a log skips cleanly, and the pure-unit tests still run.

    LOG_DIR=/path/to/logs python3 tests/test_toolkit.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pytest                       # noqa: F401
except ImportError:                     # PyPI is blocked in the sandbox; see tests/_shim.py
    import _shim as pytest

from dflog import Log, airborne_window, mix_for, trim_decomposition          # noqa: E402
from dflog.analysis import check_batch_fft, check_ekf, check_motors, check_pids  # noqa: E402
from dflog.frames import FRAME_CLASSES                                        # noqa: E402

LOG_DIR = os.environ.get("LOG_DIR", os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "reference-logs"))

# Reference log A: a 5" quad, ESC-telemetry notch, no batch logging.
# Reference log B: the same airframe with INS_LOG_BAT_MASK=1 / OPT=4, so it exercises
# the batch-IMU path as well.
LOG_A = os.environ.get("LOG_A", os.path.join(LOG_DIR, "1980-01-10 17-54-09.bin"))
LOG_B = os.environ.get("LOG_B", os.path.join(LOG_DIR, "2026-09-03 19-44-48.bin"))


def _load(path):
    if not os.path.exists(path):
        pytest.skip(f"log not available: {path}")
    return Log(path)


# ---------------------------------------------------------------- unit tests

def test_trim_decomposition_identities():
    """On a quad, each trim is the difference between two halves of the airframe."""
    mix = mix_for(1, 1)
    m = [1510.4, 1534.5, 1486.3, 1508.1]          # M1 FR, M2 RL, M3 FL, M4 RR
    tr = trim_decomposition(m, mix)
    assert tr["roll"] == pytest.approx((m[1] + m[2]) / 2 - (m[0] + m[3]) / 2, abs=1e-9)
    assert tr["pitch"] == pytest.approx((m[0] + m[2]) / 2 - (m[1] + m[3]) / 2, abs=1e-9)
    assert tr["yaw"] == pytest.approx((m[0] + m[1]) / 2 - (m[2] + m[3]) / 2, abs=1e-9)
    assert tr["residual"] == pytest.approx(0.0, abs=1e-9)


def test_trim_legacy_factors_reproduce_the_older_convention():
    """--raw-factors must reproduce the legacy convention: +1.6 / -32.5 / +25.3 us."""
    tr = trim_decomposition([1510.4, 1534.5, 1486.3, 1508.1], mix_for(1, 1, normalise=False))
    assert tr["roll"] == pytest.approx(1.6, abs=0.1)
    assert tr["pitch"] == pytest.approx(-32.5, abs=0.1)
    assert tr["yaw"] == pytest.approx(25.3, abs=0.1)


def test_unknown_frame_falls_back_to_quad_x_and_says_so():
    mix = mix_for(99, 99)
    assert mix.n == 4
    assert "UNKNOWN" in mix.label


def test_frame_class_table_covers_the_common_ones():
    for k in (1, 2, 3, 5, 7):
        assert k in FRAME_CLASSES


# ------------------------------------------------------------- parser integrity

def test_parser_resyncs_zero_bytes():
    for path in (LOG_A, LOG_B):
        log = _load(path)
        assert log.resync_bytes == 0, f"{path} needed {log.resync_bytes} resync bytes"


def test_message_count_matches_the_reference_figure():
    """Reference log A contains exactly 159,184 messages."""
    log = _load(LOG_A)
    assert log.n_messages == 159184


def test_cache_round_trips():
    log = _load(LOG_B)
    again = Log(log.path)                      # loads the cache written by the first parse
    assert again.n_messages == log.n_messages
    assert again.columns("ESC") == log.columns("ESC")


def test_instance_detection():
    log = _load(LOG_B)
    assert log.instance_field("ESC") == "Instance"
    assert log.instance_field("XKF4") == "C"
    assert log.instance_field("VIBE") == "IMU"
    assert len(log.instances("ESC")) == 4


def test_field_alias_probing():
    log = _load(LOG_B)
    assert log.field("CTUN", "BAlt", "BarAlt") is not None
    assert log.field("CTUN", "NoSuchField") is None


# --------------------------------------- reproduction of the hand-analysed numbers

def test_attitude_error_sd_matches_reference_a():
    """Reference: ATT roll err sd 0.413 deg, pitch 0.381 deg."""
    log = _load(LOG_A)
    w = airborne_window(log, method="rpm")
    att = w.clip(log.df("ATT"))
    assert float(np.std(att["Roll"] - att["DesRoll"])) == pytest.approx(0.413, abs=0.005)
    assert float(np.std(att["Pitch"] - att["DesPitch"])) == pytest.approx(0.381, abs=0.005)


def test_standing_trim_matches_reference_a():
    """Raw *channel*-ordered math, un-normalised factors: +8.3 / +12.9 / +22.6 us.

    This pins the arithmetic only. These are NOT this aircraft's roll/pitch/yaw
    trims: log A comes from a board whose SERVOn_FUNCTION puts motor 4 on output
    1, so feeding C1..C4 straight in permutes the axes. See
    `test_channel_to_motor_map_is_read_from_servo_functions` for the real
    figures, and `motor_channels()` for why.
    """
    log = _load(LOG_A)
    w = airborne_window(log, method="rpm")
    rcou = w.clip(log.df("RCOU"))
    means = [float(rcou[f"C{i}"].mean()) for i in (1, 2, 3, 4)]
    tr = trim_decomposition(means, mix_for(1, 1, normalise=False))
    assert tr["roll"] == pytest.approx(8.3, abs=0.2)
    assert tr["pitch"] == pytest.approx(12.9, abs=0.2)
    assert tr["yaw"] == pytest.approx(22.6, abs=0.2)


def test_channel_to_motor_map_is_read_from_servo_functions():
    """Log A's board maps SERVO1..4_FUNCTION = 36,33,34,35, i.e. C1 is motor 4.

    Decoded in motor order the same flight reads roll -22.6, pitch -9.2,
    yaw +5.8 us: a thrust asymmetry with essentially no standing torque, not
    the +22.6 us "yaw" that the channel-ordered arithmetic above produces.
    """
    from dflog.frames import motor_channels
    log = _load(LOG_A)
    p = log.params()
    assert motor_channels(p, 4) == ["C2", "C3", "C4", "C1"]
    w = airborne_window(log, method="rpm")
    rcou = w.clip(log.df("RCOU"))
    means = [float(rcou[c].mean()) for c in motor_channels(p, 4)]
    tr = trim_decomposition(means, mix_for(1, 1))
    assert tr["roll"] == pytest.approx(-22.6, abs=0.2)
    assert tr["pitch"] == pytest.approx(-9.2, abs=0.2)
    assert tr["yaw"] == pytest.approx(5.8, abs=0.2)


def test_motor_channels_falls_back_when_unmapped():
    from dflog.frames import motor_channels
    assert motor_channels({}, 4) is None
    assert motor_channels({f"SERVO{i}_FUNCTION": 32 + i for i in (1, 2, 3, 4)}, 4) \
        == ["C1", "C2", "C3", "C4"]


def test_ekf_mag_innovation_matches_reports():
    """Reference: log A XKF4.SM max 1.560 with 6 over 1.0; log B max 1.480 with 12."""
    for path, mx, over in ((LOG_A, 1.56, 6), (LOG_B, 1.48, 12)):
        log = _load(path)
        w = airborne_window(log, method="rpm")
        sec = check_ekf(log, w)
        sm = next(r for r in sec.results if r.name.endswith("SM innovation"))
        assert sm.evidence["value"] == pytest.approx(mx, abs=0.01)
        assert sm.evidence["samples_over_1"] == over


def test_esc_error_rate_is_a_percentage_not_an_rpm():
    """Regression: the Err% column index shifted when RPM median was added, and the
    check silently started grading raw RPM against a 5% threshold."""
    log = _load(LOG_B)
    w = airborne_window(log, method="rpm")
    sec = check_motors(log, w)
    err = next(r for r in sec.results if "DShot" in r.name)
    assert 0.0 <= err.evidence["value"] <= 100.0


def test_rpm_spread_matches_reference_b():
    """Reference: 7.48% spread on medians."""
    log = _load(LOG_B)
    w = airborne_window(log, method="rpm")
    sec = check_motors(log, w)
    spread = next(r for r in sec.results if r.name == "RPM spread")
    assert spread.evidence["value"] == pytest.approx(7.48, abs=0.05)


def test_yaw_trim_matches_reference_b():
    """Motor-ordered: roll -25.3, pitch +23.0, yaw +1.1 us.

    Superseded 2026-09-04. The earlier pin of "yaw trim +25.3" was the
    channel-ordered figure; on this board C1 is motor 4, so what was called yaw
    was really -roll. The standing torque is ~1 us; the asymmetry is thrust.
    """
    log = _load(LOG_B)
    w = airborne_window(log, method="rpm")
    sec = check_motors(log, w)
    got = {r.name: r.evidence["signed"] for r in sec.results if r.name.endswith(" trim")}
    assert got["roll trim"] == pytest.approx(-25.3, abs=0.1)
    assert got["pitch trim"] == pytest.approx(23.0, abs=0.1)
    assert got["yaw trim"] == pytest.approx(1.1, abs=0.1)


def test_notch_tracking_matches_reference_b():
    """Reference: CF/fundamental p05 0.986, p95 1.013, 0.00% above 1.5x."""
    log = _load(LOG_B)
    w = airborne_window(log, method="rpm")
    from dflog.analysis import check_notch
    sec = check_notch(log, w)
    mistrack = next(r for r in sec.results if "lock-on" in r.name)
    assert mistrack.evidence["value"] == pytest.approx(0.0, abs=0.01)


def test_batch_fft_places_the_notch_on_the_fundamental():
    """Reference: deepest attenuation at order 0.996 and 2.005, ~-34.7 dB."""
    log = _load(LOG_B)
    w = airborne_window(log, method="rpm")
    sec = check_batch_fft(log, w)
    placements = [r for r in sec.results if "placement" in r.name]
    assert len(placements) == 2
    o1 = placements[0].evidence["order"]
    o2 = placements[1].evidence["order"]
    assert o1 == pytest.approx(1.0, abs=0.02)
    assert o2 == pytest.approx(2.0, abs=0.02)


def test_no_check_raises():
    """A broken check must degrade to SKIP, never kill the report."""
    from dflog.analysis import run
    log = _load(LOG_B)
    secs = run(log)
    for s in secs:
        for r in s.results:
            assert "raised" not in r.summary, f"{s.title}: {r.summary}"


def test_windows_agree_within_a_couple_of_seconds():
    log = _load(LOG_B)
    ev = airborne_window(log, method="ev")
    rpm = airborne_window(log, method="rpm")
    assert abs(ev.duration - rpm.duration) < 3.0
    assert ev.method != rpm.method       # the method string must always be reported


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
