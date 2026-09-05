"""The check contract, and the numeric thresholds every check is judged against.

The result shape is borrowed from dronekit-la: a check never returns a bare
number, it returns a status, the evidence that produced it, and the window it
looked at. That makes results rankable, machine-readable, and - the part that
matters most in practice - impossible to quote without the number attached.

Thresholds are sourced, not invented. Provenance for each is in the
`source` field and in ../reference/thresholds.md. Where two upstream projects
disagree (e.g. compass offsets: LogAnalyzer 300/500 vs dronekit-la 100/200),
both are recorded and the stricter is used for WARN.
"""

from __future__ import annotations

__all__ = ["Result", "PASS", "WARN", "FAIL", "SKIP", "T"]

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"
_RANK = {PASS: 0, SKIP: 1, WARN: 2, FAIL: 3}


class Result:
    """One check outcome."""

    def __init__(self, name, status, summary, evidence=None, window=None,
                 source=None, severity=None):
        self.name = name
        self.status = status
        self.summary = summary                 # one sentence, always with the number in it
        self.evidence = dict(evidence or {})   # the numbers behind the verdict
        self.window = window
        self.source = source                   # where the threshold came from
        self.severity = severity if severity is not None else {PASS: 0, SKIP: 0, WARN: 10, FAIL: 20}[status]

    @property
    def rank(self):
        return _RANK[self.status]

    def line(self):
        return f"[{self.status}] {self.name}: {self.summary}"

    def to_dict(self):
        return dict(name=self.name, status=self.status, summary=self.summary,
                    evidence=_clean(self.evidence), severity=self.severity,
                    window=(self.window.t0, self.window.t1) if self.window else None,
                    source=self.source)

    def __repr__(self):
        return f"<Result {self.name} {self.status}>"


def _clean(v):
    """Make evidence JSON-safe: numpy scalars to Python, non-finite floats to None."""
    try:
        import numpy as np
    except ImportError:  # pragma: no cover
        np = None
    if np is not None and isinstance(v, np.generic):
        v = v.item()
    if isinstance(v, float) and v != v:
        return None
    if isinstance(v, float) and v in (float("inf"), float("-inf")):
        return None
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if np is not None and isinstance(v, np.ndarray):
        return [_clean(x) for x in v.tolist()]
    return v


def _t(warn, fail, source, note=""):
    return dict(warn=warn, fail=fail, source=source, note=note)


LA = "ardupilot Tools/LogAnalyzer @ bdea9be7fb~1 (removed from master 2024-08-14)"
DLA = "dronekit-la (unmaintained since 2022)"
WIKI = "ardupilot.org wiki"
MEAS = "measured on the development logs; see reference/thresholds.md"

#: Threshold registry. Every check must cite one of these rather than hardcode.
T = {
    # --- vibration -------------------------------------------------------
    # ArduPilot's own rule of thumb, in the VIBE message's units (m/s^2).
    "vibe_xy":        _t(15.0, 30.0, WIKI, "VIBE.VibeX/Y m/s^2; <15 good, >30 problematic"),
    "vibe_z":         _t(20.0, 30.0, WIKI, "VIBE.VibeZ m/s^2; Z tolerates a little more"),
    # LogAnalyzer measured 2-sigma of raw IMU accel in g over a loiter chunk.
    "vibe_2sigma_xy": _t(1.5, 3.0, LA, "2-sigma of IMU.AccX/Y in g"),
    "vibe_2sigma_z":  _t(2.0, 5.0, LA, "2-sigma of IMU.AccZ in g"),
    "clip_events":    _t(1, 20, WIKI, "VIBE.Clip counter delta over the flight; any clipping is a flag"),

    # --- IMU -------------------------------------------------------------
    "imu_match_mss":  _t(0.75, 1.5, LA, "TestIMUMatch: low-passed |acc| difference between IMUs, m/s^2"),
    "gyro_bias_dps":  _t(1.0, 3.0, MEAS, "largest-axis mean gyro rate over the airborne window, deg/s; "
                                         "a hover should average near zero"),

    # --- compass ---------------------------------------------------------
    "compass_offsets":  _t(100.0, 300.0, f"{DLA} 100/200, {LA} 300/500", "|COMPASS_OFS| vector length"),
    "compass_field_lo": _t(120.0, 100.0, f"{LA}, {DLA}", "field magnitude mGauss, low side"),
    "compass_field_hi": _t(550.0, 600.0, f"{LA}, {DLA}", "field magnitude mGauss, high side"),
    "compass_field_var": _t(0.25, 0.35, LA, "(max-min)/min of field magnitude over the flight"),
    "compass_mot_corr": _t(0.30, 0.50, MEAS, "|corr(throttle, |B|)|; above this, run COMPASS_MOT"),

    # --- EKF -------------------------------------------------------------
    # XKF4 innovation test ratios: ArduPilot rejects the measurement at 1.0.
    "ekf_innov":      _t(0.5, 1.0, DLA, "XKF4 SV/SP/SH/SM variance ratios; reject at 1.0"),
    "ekf_errRP":      _t(0.05, 0.10, MEAS, "XKF4.errRP"),
    "att_div_deg":    _t(5.0, 10.0, DLA, "attitude_estimate_divergence: |ATT - AHR2/XKF1| roll/pitch, deg"),
    "alt_div_m":      _t(4.0, 5.0, DLA, "altitude_estimate_divergence: |baro - EKF| altitude, m"),

    # --- GPS -------------------------------------------------------------
    "gps_sats":       _t(6, 5, LA, "GPS.NSats, low side"),
    "gps_hdop":       _t(3.0, 10.0, LA, "GPS.HDop"),
    "gps_nofix_pct":  _t(2.0, 10.0, MEAS, "percent of airborne time without a 3D fix"),
    "gps_glitch_speed": _t(30.0, 60.0, MEAS, "max implied ground speed between consecutive 3D fixes, m/s; "
                                             "a multirotor log jumping faster than this is a position glitch"),

    # --- power / CPU -----------------------------------------------------
    "vcc_min":        _t(4.7, 4.6, LA, "POWR.Vcc volts, low side"),
    "vcc_spread":     _t(0.3, 0.5, LA, "POWR.Vcc max-min volts"),
    "cpu_load":       _t(60.0, 80.0, MEAS, "PM.Load percent (PM.Load is percent x10 in the log)"),
    "cpu_slow_pct":   _t(6.0, 10.0, LA, "PM.NLon/PM.NL as a percent of loops"),
    "free_mem_bytes": _t(20000, 5000, MEAS, "PM.Mem minimum free bytes; scripting and logging need headroom"),
    "brownout_alt_m": _t(1.0, 3.0, LA, "TestBrownout: still armed at log end with BAlt above this = truncated in flight"),

    # --- motors ----------------------------------------------------------
    "rpm_spread_pct": _t(3.0, 8.0, MEAS, "(max-min)/mean of per-motor mean RPM, airborne"),
    "trim_us":        _t(10.0, 25.0, MEAS, "|roll/pitch/yaw trim| in us of motor output"),
    "esc_err_pct":    _t(5.0, 15.0, MEAS, "ESC.Err, bidirectional DShot error rate percent"),
    "motor_headroom": _t(0.90, 0.97, DLA, "peak output as a fraction of the MOT_SPIN_MAX ceiling"),

    # --- attitude / tune -------------------------------------------------
    "att_err_deg":    _t(5.0, 10.0, DLA, "|actual - desired| roll/pitch in degrees"),
    "rate_corr":      _t(0.6, 0.4, MEAS, "corr(desired rate, actual rate), low side"),
    "gust_event_rate": _t(0.2, 0.5, MEAS, "unrequested >2.5 deg excursions per second (see reference/thresholds.md)"),
    "lean_over_max_deg": _t(0.0, 10.0, LA, "TestPitchRollCoupling: lean angle beyond ANGLE_MAX, deg; "
                                            "LA fails at +10 deg above 2 m"),

    # --- notch / spectra -------------------------------------------------
    "notch_track":    _t(0.05, 0.15, MEAS, "|notch CF / ESC fundamental - 1|, p95"),
    "notch_mistrack_pct": _t(1.0, 5.0, MEAS, "percent of airborne time with CF > 1.5x fundamental"),
    "notch_atten_db": _t(-10.0, -6.0, WIKI, "notch attenuation at the fundamental, dB (more negative is better)"),
    "motor_peak_db":  _t(25.0, 40.0, WIKI, "peak at a motor order above the spectral floor, dB; FFT_SNR_REF "
                                           "default 25 dB, FFT_OPTIONS warns above 40 dB"),
}
