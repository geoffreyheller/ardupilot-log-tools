"""Flight segmentation: when was the aircraft actually flying?

Every number in a log analysis is only as good as its window. Spin-up and
spin-down samples wreck RPM statistics, ground handling wrecks vibration
statistics, and a notch clamped at its floor while sitting on the bench looks
exactly like a notch clamped in a descent.

`airborne_window()` picks a window and, crucially, *tells you which method it
used*. Quote that method in any report: a 4.7 % vs 6.31 % discrepancy between
two analyses of the same log turned out to be entirely a window-definition
difference, not a disagreement about the data.
"""

from __future__ import annotations

import numpy as np

__all__ = ["EVENTS", "MODES", "MODES_BY_VEHICLE", "MODE_REASONS", "airborne_window",
           "arm_window", "mode_timeline", "events", "esc_fundamental", "Window",
           "hover_chunks", "WINDOW_METHODS"]

# ArduPilot LogEvent ids (libraries/AP_Logger/AP_Logger.h, enum class LogEvent).
EVENTS = {
    10: "ARMED", 11: "DISARMED", 15: "AUTO_ARMED", 17: "LAND_COMPLETE_MAYBE",
    18: "LAND_COMPLETE", 19: "LOST_GPS", 21: "FLIP_START", 22: "FLIP_END",
    25: "SET_HOME", 26: "SET_SIMPLE_ON", 27: "SET_SIMPLE_OFF", 28: "NOT_LANDED",
    29: "SET_SUPERSIMPLE_ON", 30: "AUTOTUNE_INITIALISED", 31: "AUTOTUNE_OFF",
    32: "AUTOTUNE_RESTART", 33: "AUTOTUNE_SUCCESS", 34: "AUTOTUNE_FAILED",
    35: "AUTOTUNE_REACHED_LIMIT", 36: "AUTOTUNE_PILOT_TESTING",
    37: "AUTOTUNE_SAVEDGAINS", 38: "SAVE_TRIM", 39: "SAVEWP_ADD_WP",
    41: "FENCE_ENABLE", 42: "FENCE_DISABLE", 43: "ACRO_TRAINER_OFF",
    44: "ACRO_TRAINER_LEVELING", 45: "ACRO_TRAINER_LIMITED",
    46: "GRIPPER_GRAB", 47: "GRIPPER_RELEASE",
    49: "PARACHUTE_DISABLED", 50: "PARACHUTE_ENABLED", 51: "PARACHUTE_RELEASED",
    52: "LANDING_GEAR_DEPLOYED", 53: "LANDING_GEAR_RETRACTED",
    54: "MOTORS_EMERGENCY_STOPPED", 55: "MOTORS_EMERGENCY_STOP_CLEARED",
    56: "MOTORS_INTERLOCK_DISABLED", 57: "MOTORS_INTERLOCK_ENABLED",
    58: "ROTOR_RUNUP_COMPLETE", 59: "ROTOR_SPEED_BELOW_CRITICAL",
    60: "EKF_ALT_RESET", 61: "LAND_CANCELLED_BY_PILOT", 62: "EKF_YAW_RESET",
    63: "AVOIDANCE_ADSB_ENABLE", 64: "AVOIDANCE_ADSB_DISABLE",
    65: "AVOIDANCE_PROXIMITY_ENABLE", 66: "AVOIDANCE_PROXIMITY_DISABLE",
    67: "GPS_PRIMARY_CHANGED", 68: "WINCH_RELAXED", 69: "WINCH_LENGTH_CONTROL",
    70: "WINCH_RATE_CONTROL", 71: "ZIGZAG_STORE_A", 72: "ZIGZAG_STORE_B",
    73: "LAND_REPO_ACTIVE", 74: "STANDBY_ENABLE", 75: "STANDBY_DISABLE",
    76: "FENCE_ALT_MAX_ENABLE", 77: "FENCE_ALT_MAX_DISABLE",
    78: "FENCE_CIRCLE_ENABLE", 79: "FENCE_CIRCLE_DISABLE",
    80: "FENCE_ALT_MIN_ENABLE", 81: "FENCE_ALT_MIN_DISABLE",
    82: "FENCE_POLYGON_ENABLE", 83: "FENCE_POLYGON_DISABLE",
    85: "EK3_SOURCES_SET_TO_PRIMARY", 86: "EK3_SOURCES_SET_TO_SECONDARY",
    87: "EK3_SOURCES_SET_TO_TERTIARY", 90: "AIRSPEED_PRIMARY_CHANGED",
    163: "SURFACED", 164: "NOT_SURFACED", 165: "BOTTOMED", 166: "NOT_BOTTOMED",
    167: "EKF_MAG_OFFSETS_SAVED",
}

# Mode numbers are vehicle-specific. MODES is the copter table (this toolkit's focus);
# mode_timeline() picks the table from the firmware string when it can.
MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
    15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW",
    24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL", 28: "TURTLE",
}
_PLANE_MODES = {
    0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING", 4: "ACRO", 5: "FBWA",
    6: "FBWB", 7: "CRUISE", 8: "AUTOTUNE", 10: "AUTO", 11: "RTL", 12: "LOITER",
    13: "TAKEOFF", 14: "AVOID_ADSB", 15: "GUIDED", 16: "INITIALISING",
    17: "QSTABILIZE", 18: "QHOVER", 19: "QLOITER", 20: "QLAND", 21: "QRTL",
    22: "QAUTOTUNE", 23: "QACRO", 24: "THERMAL", 25: "LOITER_ALT_QLAND", 26: "AUTOLAND",
}
_ROVER_MODES = {
    0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER", 6: "FOLLOW",
    7: "SIMPLE", 8: "DOCK", 9: "CIRCLE", 10: "AUTO", 11: "RTL", 12: "SMART_RTL",
    15: "GUIDED", 16: "INITIALISING",
}
_SUB_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 7: "CIRCLE",
    9: "SURFACE", 16: "POSHOLD", 19: "MANUAL", 20: "MOTOR_DETECT", 21: "SURFTRAK",
}
MODES_BY_VEHICLE = {"ArduCopter": MODES, "ArduPlane": _PLANE_MODES, "ArduRover": _ROVER_MODES,
                    "Rover": _ROVER_MODES, "ArduSub": _SUB_MODES}

# MODE.Rsn (libraries/AP_Vehicle/ModeReason.h).
MODE_REASONS = {
    0: "UNKNOWN", 1: "RC_COMMAND", 2: "GCS_COMMAND", 3: "RADIO_FAILSAFE", 4: "BATTERY_FAILSAFE",
    5: "GCS_FAILSAFE", 6: "EKF_FAILSAFE", 7: "GPS_GLITCH", 8: "MISSION_END",
    9: "THROTTLE_LAND_ESCAPE", 10: "FENCE_BREACHED", 11: "TERRAIN_FAILSAFE", 12: "BRAKE_TIMEOUT",
    13: "FLIP_COMPLETE", 14: "AVOIDANCE", 15: "AVOIDANCE_RECOVERY", 16: "THROW_COMPLETE",
    17: "TERMINATE", 18: "TOY_MODE", 19: "CRASH_FAILSAFE", 20: "SOARING_FBW_B_WITH_MOTOR_RUNNING",
    21: "SOARING_THERMAL_DETECTED", 22: "SOARING_THERMAL_ESTIMATE_DETERIORATED",
    23: "VTOL_FAILED_TRANSITION", 24: "VTOL_FAILED_TAKEOFF", 25: "FAILSAFE", 26: "INITIALISED",
    27: "SURFACE_COMPLETE", 28: "BAD_DEPTH", 29: "LEAK_FAILSAFE", 30: "SERVOTEST", 31: "STARTUP",
    32: "SCRIPTING", 33: "UNAVAILABLE", 34: "AUTOROTATION_START", 35: "AUTOROTATION_BAILOUT",
    36: "SOARING_ALT_TOO_HIGH", 37: "SOARING_ALT_TOO_LOW", 38: "SOARING_DRIFT_EXCEEDED",
    39: "RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL", 40: "RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND",
    41: "MISSION_CMD", 42: "FRSKY_COMMAND", 43: "FENCE_RETURN_PREVIOUS_MODE",
    44: "QRTL_INSTEAD_OF_RTL", 45: "AUTO_RTL_EXIT", 46: "LOITER_ALT_REACHED_QLAND",
    47: "LOITER_ALT_IN_VTOL", 48: "RADIO_FAILSAFE_RECOVERY", 49: "QLAND_INSTEAD_OF_RTL",
    50: "DEADRECKON_FAILSAFE", 51: "MODE_TAKEOFF_FAILSAFE", 52: "DDS_COMMAND", 53: "AUX_FUNCTION",
    54: "FIXED_WING_AUTOLAND", 55: "FENCE_REENABLE",
}

WINDOW_METHODS = ("auto", "ev", "rpm", "throttle", "arm", "none")


class Window:
    """A time window, plus a record of how it was chosen."""

    __slots__ = ("t0", "t1", "method", "note")

    def __init__(self, t0, t1, method, note=""):
        self.t0, self.t1 = float(t0), float(t1)
        self.method, self.note = method, note

    @property
    def duration(self):
        return self.t1 - self.t0

    def mask(self, t):
        t = np.asarray(t, dtype=float)
        return (t >= self.t0) & (t <= self.t1)

    def clip(self, df, tcol="t"):
        """Rows of a DataFrame inside the window."""
        if df is None or getattr(df, "empty", True) or tcol not in df.columns:
            return df
        return df[self.mask(df[tcol].values)]

    def to_dict(self):
        return dict(t0=self.t0, t1=self.t1, duration_s=self.duration, method=self.method, note=self.note)

    def __repr__(self):
        return f"<Window {self.t0:.1f}-{self.t1:.1f}s ({self.duration:.1f}s) via {self.method}>"


def events(log):
    """[(t, id, name)] from EV records."""
    d = log.df("EV")
    if d.empty:
        return []
    return [(float(t), int(i), EVENTS.get(int(i), f"EV_{int(i)}"))
            for t, i in zip(d["t"], d["Id"])]


def mode_table(log):
    """The mode-number table for this log's vehicle (copter if unknown, and said so)."""
    veh = log.vehicle() if hasattr(log, "vehicle") else ""
    return MODES_BY_VEHICLE.get(veh, MODES), veh


def mode_timeline(log):
    """[(t, mode_num, mode_name, reason)] from MODE records."""
    d = log.df("MODE")
    if d.empty:
        return []
    table, _veh = mode_table(log)
    out = []
    for _, r in d.iterrows():
        num = int(r.get("ModeNum", r.get("Mode", -1)))
        out.append((float(r["t"]), num, table.get(num, f"MODE_{num}"), int(r.get("Rsn", -1))))
    return out


def esc_fundamental(log, t=None):
    """Fleet-mean motor fundamental in Hz (ESC RPM / 60), resampled onto `t`.

    Returns (t, hz) or (None, None) if the log has no ESC telemetry. This is
    the ground truth every notch and motor-balance check is measured against.
    """
    inst = log.instances("ESC")
    if not inst:
        return None, None
    if t is None:
        base = max(inst.values(), key=len)
        t = base["t"].values
    t = np.asarray(t, dtype=float)
    series = []
    for v in inst.values():
        if "RPM" not in v.columns or len(v) < 2:
            continue
        series.append(np.interp(t, v["t"].values, v["RPM"].values / 60.0))
    if not series:
        return None, None
    return t, np.mean(series, axis=0)


def arm_window(log):
    """Armed period from EV ARMED/DISARMED, else the whole log."""
    ev = events(log)
    arm = [t for t, i, _ in ev if i == 10]
    dis = [t for t, i, _ in ev if i == 11]
    if arm:
        t0 = arm[0]
        t1 = max([d for d in dis if d > t0], default=log.duration()[1])
        return Window(t0, t1, "EV ARMED->DISARMED")
    lo, hi = log.duration()
    return Window(lo, hi, "whole log (no ARMED event)")


def airborne_window(log, method="auto", hz_floor=90.0, thr_floor=0.15, pad=0.0):
    """Best available airborne window.

    method:
      "auto"  - try ev, then rpm, then throttle, then arm
      "ev"    - EV NOT_LANDED (28) .. LAND_COMPLETE (18). Most authoritative.
      "rpm"   - fleet-mean ESC fundamental above `hz_floor`. Use when comparing
                flights on motor/notch metrics: it is defined identically
                regardless of what the FC's land detector believed.
      "arm"   - EV ARMED .. DISARMED (includes ground time; rarely what you want)
      "throttle" - CTUN.ThO above `thr_floor`
      "none"  - the whole log
      "T0:T1" - explicit seconds since boot, e.g. "120:180"

    `pad` trims that many seconds off each end, useful for excluding the
    takeoff and landing transients from steady-state statistics.

    If the requested method cannot be applied (e.g. "rpm" on a log with no ESC
    telemetry) the fallback is the whole log and the method string says so -
    the caller can never mistake a fallback for the window it asked for.
    """
    lo, hi = log.duration()
    if lo is None:
        return Window(0.0, 0.0, "no timestamped messages in log")
    if isinstance(method, str) and ":" in method:
        a, b = method.split(":", 1)
        t0 = float(a) if a.strip() else lo
        t1 = float(b) if b.strip() else hi
        if t1 <= t0:
            raise ValueError(f"window {method!r}: end must be after start")
        return Window(t0, t1, f"explicit {t0:g}:{t1:g} s")
    if method == "none":
        return Window(lo, hi, "whole log (requested)")
    order = [method] if method != "auto" else ["ev", "rpm", "throttle", "arm"]
    for m in order:
        w = _try_window(log, m, hz_floor, thr_floor)
        if w is not None and w.duration > 1.0:
            if pad:
                if 2 * pad >= w.duration:
                    raise ValueError(f"pad {pad}s removes the whole {w.duration:.1f}s window")
                w = Window(w.t0 + pad, w.t1 - pad, w.method, f"padded {pad}s each end")
            return w
    return Window(lo, hi, f"whole log (FALLBACK: method {method!r} found no airborne signal)")


def _try_window(log, method, hz_floor, thr_floor):
    if method == "ev":
        ev = events(log)
        ups = [t for t, i, _ in ev if i == 28]
        downs = [t for t, i, _ in ev if i == 18]
        if not ups:
            return None
        t0 = ups[0]
        t1 = min([d for d in downs if d > t0], default=None)
        if t1 is None:
            t1 = log.duration()[1]
        return Window(t0, t1, "EV NOT_LANDED->LAND_COMPLETE")

    if method == "rpm":
        t, hz = esc_fundamental(log)
        if hz is None:
            return None
        m = hz > hz_floor
        if not m.any():
            return None
        return Window(t[m].min(), t[m].max(),
                      f"ESC fundamental > {hz_floor:g} Hz",
                      "same definition regardless of land-detector state")

    if method == "throttle":
        d = log.df("CTUN")
        col = "ThO" if "ThO" in d.columns else ("ThrOut" if "ThrOut" in d.columns else None)
        if d.empty or col is None:
            return None
        v = d[col].values
        if np.nanmax(v) > 1.5:          # legacy 0-1000 scale
            v = v / 1000.0
        m = v > thr_floor
        if not m.any():
            return None
        return Window(d["t"].values[m].min(), d["t"].values[m].max(),
                      f"CTUN.{col} > {thr_floor:g}")

    if method == "arm":
        return arm_window(log)
    return None


def hover_chunks(log, min_seconds=10.0, rc_quiet=True, modes=("LOITER", "ALT_HOLD", "POSHOLD")):
    """Steady-hover windows: the precondition for most noise statistics.

    Ported from LogAnalyzer's `DataflashLogHelper.findLoiterChunks`. Vibration,
    FFT and motor-balance numbers taken over a whole flight mix hover with
    aggressive manoeuvring and are not comparable between flights; taken over
    a quiet hover chunk they are.

    `rc_quiet` additionally requires roll/pitch/yaw sticks near centre, so
    pilot input is excluded. Returns a list of Window.
    """
    tl = mode_timeline(log)
    if not tl:
        return []
    end = log.duration()[1]
    spans = []
    for i, (t, _num, name, _rsn) in enumerate(tl):
        t_end = tl[i + 1][0] if i + 1 < len(tl) else end
        if name in modes and (t_end - t) >= min_seconds:
            spans.append((t, t_end, name))
    if not spans:
        return []
    if not rc_quiet:
        return [Window(a, b, f"mode {m} >= {min_seconds:g}s") for a, b, m in spans]

    rcin = log.df("RCIN")
    out = []
    for a, b, m in spans:
        if rcin.empty:
            out.append(Window(a, b, f"mode {m} >= {min_seconds:g}s (RC not logged)"))
            continue
        seg = rcin[(rcin["t"] >= a) & (rcin["t"] <= b)]
        quiet = np.ones(len(seg), dtype=bool)
        for ch in ("C1", "C2", "C4"):          # roll, pitch, yaw sticks
            if ch in seg.columns:
                quiet &= np.abs(seg[ch].values - 1500) < 40
        if not quiet.any():
            continue
        t = seg["t"].values
        idx = np.flatnonzero(quiet)
        splits = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        for run in splits:
            if len(run) < 2:
                continue
            t0, t1 = t[run[0]], t[run[-1]]
            if t1 - t0 >= min_seconds:
                out.append(Window(t0, t1, f"mode {m}, sticks centred"))
    return out
