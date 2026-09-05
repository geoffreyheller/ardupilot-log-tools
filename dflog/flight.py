"""Flight segmentation: when was the aircraft actually flying?

Every number in a log analysis is only as good as its window. Spin-up and
spin-down samples wreck RPM statistics, ground handling wrecks vibration
statistics, and a notch clamped at its floor while sitting on the bench looks
exactly like a notch clamped in a descent.

`airborne_window()` picks a window and, crucially, *tells you which method it
used*. Quote that method in any report: a 4.7 % vs 6.31 % discrepancy between
two analyses of the same log turned out to be entirely a window-definition
difference, not a disagreement about the data.

A log can hold more than one flight. `flights()` returns every one of them;
`airborne_window()` returns exactly one - never the span across the ground time
between two - and names which, so a reader of the report can tell that a second
flight exists and was not analysed.
"""

from __future__ import annotations

import numpy as np

__all__ = ["EVENTS", "MODES", "MODES_BY_VEHICLE", "MODE_REASONS", "airborne_window",
           "arm_window", "flights", "mode_timeline", "events", "esc_fundamental", "Window",
           "hover_chunks", "WINDOW_METHODS", "GAP_SECONDS", "MIN_SECONDS"]

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

    __slots__ = ("t0", "t1", "method", "note", "index", "segments", "scope")

    def __init__(self, t0, t1, method, note="", index=None, segments=None, scope=None):
        self.t0, self.t1 = float(t0), float(t1)
        self.method, self.note = method, note
        # Which flight this is, and what else the log holds. `index` is 1-based, or None
        # when the window is not a single flight segment (explicit, whole-log, fallback).
        # `segments` is [(t0, t1)] for every flight in the log, or None when unknown.
        # `scope` is "all" when the caller is analysing every flight in turn, so a check
        # can tell "one of two" from "both, one at a time".
        self.index, self.segments, self.scope = index, segments, scope

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
        d = dict(t0=self.t0, t1=self.t1, duration_s=self.duration, method=self.method, note=self.note)
        if self.segments is not None:
            d["n_flights"] = len(self.segments)
            d["flight_index"] = self.index
        return d

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


# --------------------------------------------------------------- segmentation
#
# A log can hold more than one flight: take off, land, disarm, walk out, re-arm,
# take off again, all in one file. Before this existed the same idea was written
# three separate times - min..max of an airborne mask for `rpm` and `throttle`,
# first-up-to-first-down for `ev`, first-arm-to-last-disarm for `arm` - and none
# of the three knew a second flight could exist. The mask versions *spanned* the
# ground time between flights (on the log in issue #1, 37 % of the "airborne"
# window was the aircraft sitting on the ground, which produced two confident
# FAILs in the notch check); the event versions silently dropped every flight
# after the first. Every detector now goes through one of the two helpers below,
# so there is one definition of "a flight" and one place to fix.

# Window-construction parameters. These are not check thresholds - they do not
# judge an aircraft, they define a window - so they live here as keyword defaults
# rather than in checks.py::T. See reference/thresholds.md.
GAP_SECONDS = 10.0      # ground time below this is a bounce or a dip, not a new flight
MIN_SECONDS = 5.0       # anything shorter is a bench spin-up, not a flight


def _coalesce(segs, gap_seconds, min_seconds):
    """Drop segments shorter than `min_seconds`, then merge any that remain closer
    together than `gap_seconds`.

    The order is deliberate: dropping first means a two-second spin-up on the bench
    cannot glue itself onto the real flight and drag the window's start backwards.
    Merging afterwards is what keeps a bounced landing (LAND_COMPLETE then NOT_LANDED
    a second later) one flight rather than two.
    """
    kept = [s for s in segs if s[1] - s[0] >= min_seconds]
    if not kept:
        return []
    out = [list(kept[0])]
    for t0, t1, note in kept[1:]:
        if t0 - out[-1][1] < gap_seconds:
            out[-1][1] = max(out[-1][1], t1)
            out[-1][2] = out[-1][2] or note
        else:
            out.append([t0, t1, note])
    return [tuple(s) for s in out]


def _pairs(ev, up_id, down_id, end, gap_seconds, min_seconds):
    """Segments from paired up/down events, in time order.

    An up-id opens a segment if none is open; a down-id closes the open one. Repeated
    up-ids while already airborne are ignored - ArduPilot emits LAND_COMPLETE_MAYBE and
    NOT_LANDED in quick succession around a touchdown. A segment still open at `end` is
    closed there and says so, rather than being silently dropped or silently extended.
    """
    segs, t0 = [], None
    for t, i, _name in sorted(ev, key=lambda e: e[0]):
        if i == up_id and t0 is None:
            t0 = t
        elif i == down_id and t0 is not None:
            segs.append((t0, t, ""))
            t0 = None
    if t0 is not None:
        segs.append((t0, float(end), "still airborne at the end of log; segment closed there"))
    return _coalesce(segs, gap_seconds, min_seconds)


def _runs(t, mask, gap_seconds, min_seconds):
    """Segments from a boolean airborne mask over time base `t`.

    Split where consecutive True samples are more than `gap_seconds` apart, which covers
    both a genuine False region (the aircraft on the ground) and a hole in the telemetry:
    a stream that simply stops during ground time would otherwise read as one flight.
    """
    t = np.asarray(t, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    order = np.argsort(t, kind="stable")
    t, mask = t[order], mask[order]
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    tt = t[idx]
    breaks = np.flatnonzero(np.diff(tt) > gap_seconds) + 1
    segs = [(float(r[0]), float(r[-1]), "") for r in np.split(tt, breaks) if r.size]
    return _coalesce(segs, gap_seconds, min_seconds)


def _segments(log, method, hz_floor, thr_floor, gap_seconds, min_seconds):
    """[(t0, t1, note)] for one detector, or None if the detector cannot be applied.

    None and [] are different answers: None is "this log carries no ESC telemetry", []
    is "it does, and the aircraft never flew". Only the first is a fallback.
    """
    if method == "ev":
        ev = events(log)
        if not any(i == 28 for _t, i, _n in ev):
            return None
        return _pairs(ev, 28, 18, log.duration()[1], gap_seconds, min_seconds)

    if method == "arm":
        ev = events(log)
        if not any(i == 10 for _t, i, _n in ev):
            return None
        return _pairs(ev, 10, 11, log.duration()[1], gap_seconds, min_seconds)

    if method == "rpm":
        t, hz = esc_fundamental(log)
        if hz is None:
            return None
        return _runs(t, hz > hz_floor, gap_seconds, min_seconds)

    if method == "throttle":
        d = log.df("CTUN")
        col = "ThO" if "ThO" in d.columns else ("ThrOut" if "ThrOut" in d.columns else None)
        if d.empty or col is None:
            return None
        v = d[col].values
        if np.nanmax(v) > 1.5:          # legacy 0-1000 scale
            v = v / 1000.0
        return _runs(d["t"].values, v > thr_floor, gap_seconds, min_seconds)

    return None


def _describe(log, method, hz_floor, thr_floor):
    """(method string, note) for a detector.

    These strings are what this tool has always printed. They appear in pinned
    assertions, in reference/json-output.md and in every report anyone has already
    written, so they do not change.
    """
    if method == "ev":
        return "EV NOT_LANDED->LAND_COMPLETE", ""
    if method == "arm":
        return "EV ARMED->DISARMED", ""
    if method == "rpm":
        return f"ESC fundamental > {hz_floor:g} Hz", "same definition regardless of land-detector state"
    if method == "throttle":
        col = "ThO" if "ThO" in log.df("CTUN").columns else "ThrOut"
        return f"CTUN.{col} > {thr_floor:g}", ""
    return method, ""


def flights(log, method="auto", hz_floor=90.0, thr_floor=0.15,
            min_seconds=MIN_SECONDS, gap_seconds=GAP_SECONDS):
    """Every flight segment in the log, in time order, as a list of Window.

    Each Window carries `index` (1-based), `segments` (every flight in the log, so a
    caller can report the ones it did not analyse) and a method string naming the
    detector *and the segment*: "ESC fundamental > 90 Hz, flight 2 of 2". The suffix is
    omitted when there is only one flight, so a single-flight log's method string is
    exactly what it has always been.

    method: "auto" tries ev, rpm, throttle, arm in that order and returns the first that
    finds anything; the rest select one detector. Returns [] when the requested detector
    cannot be applied at all - that is a fallback, and the caller must say so.

    `gap_seconds` is the load-bearing parameter: ground time shorter than this is a
    bounced landing or a mid-flight dip below the detector's floor, not a new flight.
    """
    order = ["ev", "rpm", "throttle", "arm"] if method == "auto" else [method]
    for m in order:
        segs = _segments(log, m, hz_floor, thr_floor, gap_seconds, min_seconds)
        if not segs:
            continue
        base, note = _describe(log, m, hz_floor, thr_floor)
        spans = [(t0, t1) for t0, t1, _n in segs]
        out = []
        for k, (t0, t1, seg_note) in enumerate(segs, 1):
            meth = base if len(segs) == 1 else f"{base}, flight {k} of {len(segs)}"
            out.append(Window(t0, t1, meth, "; ".join(x for x in (note, seg_note) if x),
                              index=k, segments=spans))
        return out
    return []


def arm_window(log):
    """Armed period from EV ARMED/DISARMED, else the whole log.

    Deliberately the *whole* armed span, first ARMED to last DISARMED, ground time and
    any intervening flights included - that is what "armed" means. For one flight at a
    time use `airborne_window(log, method="arm")` or `flights(log, method="arm")`.
    """
    ev = events(log)
    arm = [t for t, i, _ in ev if i == 10]
    dis = [t for t, i, _ in ev if i == 11]
    if arm:
        t0 = arm[0]
        t1 = max([d for d in dis if d > t0], default=log.duration()[1])
        return Window(t0, t1, "EV ARMED->DISARMED")
    lo, hi = log.duration()
    return Window(lo, hi, "whole log (no ARMED event)")


def airborne_window(log, method="auto", hz_floor=90.0, thr_floor=0.15, pad=0.0, flight=None):
    """Best available airborne window: one flight, and a record of how it was chosen.

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

    On a log holding more than one flight this returns **one** of them, never the span
    across the ground time between them. `flight=k` (1-based) picks one; the default is
    the longest, ties broken to the earliest. Out of range is a ValueError naming how
    many there are, and `flight` alongside "none" or an explicit "T0:T1" is a ValueError
    too - they contradict each other. `flights(log)` returns them all.

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
        if flight is not None:
            raise ValueError(f"window {method!r} and flight {flight!r} contradict each other: "
                             "an explicit window already says which seconds to analyse")
        a, b = method.split(":", 1)
        t0 = float(a) if a.strip() else lo
        t1 = float(b) if b.strip() else hi
        if t1 <= t0:
            raise ValueError(f"window {method!r}: end must be after start")
        return Window(t0, t1, f"explicit {t0:g}:{t1:g} s")
    if method == "none":
        if flight is not None:
            raise ValueError(f"window 'none' and flight {flight!r} contradict each other: "
                             "'none' is the whole log, every flight included")
        return Window(lo, hi, "whole log (requested)")

    fl = flights(log, method=method, hz_floor=hz_floor, thr_floor=thr_floor)
    if fl:
        if flight is None:
            # Longest, ties to the earliest: the flight most likely to be of interest,
            # and it makes `ev` and `rpm` agree on the same log.
            w = max(fl, key=lambda x: (x.duration, -x.t0))
        elif isinstance(flight, bool) or not isinstance(flight, int) or not 1 <= flight <= len(fl):
            raise ValueError(f"flight {flight!r}: this log has {len(fl)} flight(s) ("
                             + ", ".join(f"{x.t0:.1f}-{x.t1:.1f} s" for x in fl) + ")")
        else:
            w = fl[flight - 1]
        if w.duration > 1.0:
            if pad:
                if 2 * pad >= w.duration:
                    raise ValueError(f"pad {pad}s removes the whole {w.duration:.1f}s window")
                w = Window(w.t0 + pad, w.t1 - pad, w.method, f"padded {pad}s each end",
                           index=w.index, segments=w.segments)
            return w
    if flight is not None:
        raise ValueError(f"flight {flight!r}: this log has 0 flight(s) that method {method!r} "
                         "can find, so there is nothing to select")
    return Window(lo, hi, f"whole log (FALLBACK: method {method!r} found no airborne signal)")


def hover_chunks(log, min_seconds=10.0, rc_quiet=True, modes=("LOITER", "ALT_HOLD", "POSHOLD")):
    """Steady-hover windows: the precondition for most noise statistics.

    Ported from LogAnalyzer's `DataflashLogHelper.findLoiterChunks`. Vibration,
    FFT and motor-balance numbers taken over a whole flight mix hover with
    aggressive manoeuvring and are not comparable between flights; taken over
    a quiet hover chunk they are.

    `rc_quiet` additionally requires roll/pitch/yaw sticks near centre, so
    pilot input is excluded. Returns a list of Window.

    Chunks are clipped to a single flight. A mode that never changes across a landing
    would otherwise yield one "hover" chunk covering two flights and the ground time
    between them, which is exactly the window error this module exists to prevent.
    """
    tl = mode_timeline(log)
    if not tl:
        return []
    end = log.duration()[1]
    fl = flights(log)
    spans = []
    for i, (t, _num, name, _rsn) in enumerate(tl):
        t_end = tl[i + 1][0] if i + 1 < len(tl) else end
        if name not in modes:
            continue
        for a, b in ([(t, t_end)] if not fl else
                     [(max(t, f.t0), min(t_end, f.t1)) for f in fl]):
            if (b - a) >= min_seconds:
                spans.append((a, b, name))
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
