"""Motor-mix geometry and the standing-trim decomposition.

The headline tool here is `trim_decomposition()`. Raw "RPM spread" or
"RCOU spread" is a poor summary statistic: it tells you the motors disagree
but not *how*, and a bent prop, an aft CG and a twisted arm all show up as
"spread". Projecting the standing per-motor deviation onto the frame's own
roll / pitch / yaw mix factors separates them, and each axis then points at a
different physical cause:

    roll / pitch trim  ->  thrust asymmetry: CG offset, bent or damaged blade,
                           motor mount height, wind
    yaw trim           ->  *torque* asymmetry: motor mount rotation, arm twist,
                           a blade whose airfoil or twist is wrong (a
                           straightened blade recovers thrust but not drag
                           torque, so roll fixes and yaw does not)

Convention. A trim figure is the full-scale swing in microseconds across that
axis:

    trim = 2 * (dev . f) / (f . f)

where `dev` is each motor's mean output minus the fleet mean and `f` is that
axis's mix-factor vector, normalised to unit peak exactly as ArduPilot's
`AP_MotorsMatrix::normalise_rpy_factors()` does. On a quad each trim is then
literally the difference between two halves of the airframe:

    roll  trim = mean(left motors)  - mean(right motors)
    pitch trim = mean(front motors) - mean(rear motors)
    yaw   trim = mean(CCW motors)   - mean(CW motors)

Reading the signs: pitch trim negative => rear pair working harder => CG aft.
Yaw trim non-zero => the airframe makes a standing torque the controller
fights continuously.

NOTE: some older analyses use un-normalised cos() factors (+/-0.7071 for a
quad X), which makes their roll and pitch figures 1.4142x larger than the ones
produced here; yaw is identical either way. Pass `normalise=False` to reproduce
those numbers exactly.
"""

from __future__ import annotations

import numpy as np

__all__ = ["MotorMix", "mix_for", "trim_decomposition", "motor_channels",
           "FRAME_CLASSES", "FRAME_TYPES"]

FRAME_CLASSES = {
    0: "Undefined", 1: "Quad", 2: "Hexa", 3: "Octa", 4: "OctaQuad", 5: "Y6",
    6: "Heli", 7: "Tri", 8: "SingleCopter", 9: "CoaxCopter", 10: "BiCopter",
    11: "Heli_Dual", 12: "DodecaHexa", 13: "HeliQuad", 14: "Deca",
    15: "Scripting Matrix", 16: "6DoF Scripting", 17: "Dynamic Scripting Matrix",
}

FRAME_TYPES = {
    0: "Plus", 1: "X", 2: "V", 3: "H", 4: "V-Tail", 5: "A-Tail",
    10: "Y6B", 11: "Y6F", 12: "BetaFlightX", 13: "DJIX", 14: "ClockwiseX",
    15: "I", 16: "NYT-Plus", 17: "NYT-X", 18: "BetaFlightXReversed", 19: "Y4",
}

CCW, CW = 1.0, -1.0

# (motor angle in degrees clockwise from nose, yaw factor).
# Angles follow AP_MotorsMatrix::add_motor: roll = cos(angle+90), pitch = cos(angle).
# Index in the list == ArduPilot motor number - 1 == RCOU channel C<n>.
_GEOMETRY = {
    ("Quad", "Plus"):  [(90, CCW), (-90, CCW), (0, CW), (180, CW)],
    ("Quad", "X"):     [(45, CCW), (-135, CCW), (-45, CW), (135, CW)],
    ("Quad", "H"):     [(45, CW), (-135, CW), (-45, CCW), (135, CCW)],
    ("Quad", "V"):     [(45, 0.7981), (-135, 1.0000), (-45, -0.7981), (135, -1.0000)],
    ("Quad", "BetaFlightX"): [(45, CCW), (135, CW), (-135, CCW), (-45, CW)],
    ("Quad", "DJIX"):  [(45, CCW), (135, CW), (-135, CCW), (-45, CW)],
    ("Quad", "ClockwiseX"): [(45, CCW), (135, CW), (-135, CCW), (-45, CW)],
    ("Hexa", "Plus"):  [(0, CW), (180, CCW), (-120, CW), (60, CCW), (-60, CCW), (120, CW)],
    ("Hexa", "X"):     [(90, CW), (-90, CCW), (-30, CW), (150, CCW), (30, CCW), (-150, CW)],
    ("Octa", "Plus"):  [(0, CW), (180, CW), (45, CCW), (135, CCW),
                        (-45, CCW), (-135, CCW), (-90, CW), (90, CW)],
    ("Octa", "X"):     [(22.5, CW), (-157.5, CW), (67.5, CCW), (157.5, CCW),
                        (-22.5, CCW), (-112.5, CCW), (-67.5, CW), (112.5, CW)],
    ("Deca", "X"):     [(36, CCW), (-36, CW), (-108, CCW), (180, CW),
                        (108, CCW), (72, CW), (0, CCW), (-72, CW),
                        (-144, CCW), (144, CW)],
}
# Quad-X is by far the most common; alias the frame types that share its layout.
_GEOMETRY[("Quad", "X-reversed")] = _GEOMETRY[("Quad", "X")]


class MotorMix:
    """Roll / pitch / yaw mix factors for one frame, in ArduPilot motor order."""

    def __init__(self, angles_yaw, label="custom", normalise=True):
        ang = np.array([a for a, _ in angles_yaw], dtype=float)
        self.label = label
        self.n = len(ang)
        self.angles = ang
        self.yaw = np.array([y for _, y in angles_yaw], dtype=float)
        self.roll = np.cos(np.radians(ang + 90.0))
        self.pitch = np.cos(np.radians(ang))
        self.normalised = normalise
        if not normalise:
            return
        # Normalise roll/pitch to unit peak, as ArduPilot does internally.
        for name in ("roll", "pitch"):
            v = getattr(self, name)
            peak = np.abs(v).max()
            if peak > 0:
                setattr(self, name, v / peak)

    @property
    def channels(self):
        """RCOU column names carrying the motor outputs, in motor order."""
        return [f"C{i + 1}" for i in range(self.n)]

    def axes(self):
        return {"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw}

    def __repr__(self):
        return f"<MotorMix {self.label} n={self.n}>"


def mix_for(frame_class=1, frame_type=1, n_motors=None, normalise=True):
    """MotorMix from FRAME_CLASS / FRAME_TYPE params. Falls back to Quad-X.

    Pass ints (as they appear in the log's PARM records) or the string names.
    """
    cls = FRAME_CLASSES.get(int(frame_class), frame_class) if not isinstance(frame_class, str) else frame_class
    typ = FRAME_TYPES.get(int(frame_type), frame_type) if not isinstance(frame_type, str) else frame_type
    geo = _GEOMETRY.get((cls, typ))
    if geo is None:
        geo = _GEOMETRY[("Quad", "X")]
        label = f"{cls}/{typ} (UNKNOWN - assumed Quad/X)"
    else:
        label = f"{cls}/{typ}"
    if n_motors and n_motors != len(geo):
        label += f" [log shows {n_motors} motors, geometry has {len(geo)}]"
    return MotorMix(geo, label=label, normalise=normalise)



def motor_channels(params, n_motors, max_out=16):
    """RCOU column names in ArduPilot MOTOR order, from `SERVOn_FUNCTION`.

    `RCOU.C<n>` is servo *output* n, not motor n. ArduPilot's motor numbering is
    geometry (motor 1 is front-right on a quad X); which output drives it is
    wiring, declared by `SERVOn_FUNCTION` = 32 + motor number (33 = motor 1 ...
    36 = motor 4, then 37..40 for motors 5-8). Many 4-in-1 AIO boards ship a
    non-sequential default so the ESC connector matches Betaflight order - e.g.
    `SERVO1_FUNCTION=36, SERVO2=33, SERVO3=34, SERVO4=35`, where C1 is motor 4.

    Assuming C<n> == motor n on such a board silently permutes the trim
    decomposition: on a quad X it reports true yaw as roll, true roll as -yaw
    and true pitch as -pitch, which turns a pure thrust asymmetry into an
    apparent standing yaw torque. Always go through this.

    Returns None when the parameters carry no motor functions at all, so the
    caller can fall back to C1..Cn and say it is assuming the identity map.
    """
    _MOTOR_FN = {33: 1, 34: 2, 35: 3, 36: 4, 37: 5, 38: 6, 39: 7, 40: 8,
                 82: 9, 83: 10, 84: 11, 85: 12}
    chan_for = {}
    for out in range(1, max_out + 1):
        fn = params.get(f"SERVO{out}_FUNCTION")
        if fn is None:
            continue
        m = _MOTOR_FN.get(int(fn))
        if m is not None and m not in chan_for:
            chan_for[m] = out
    if not chan_for:
        return None
    if not all(m in chan_for for m in range(1, n_motors + 1)):
        return None
    return [f"C{chan_for[m]}" for m in range(1, n_motors + 1)]


def trim_decomposition(means, mix):
    """Split standing per-motor outputs into throttle + roll/pitch/yaw trim.

    `means`: per-motor mean output (µs, or RPM, or Hz — any linear unit).
    Returns dict with `throttle`, `roll`, `pitch`, `yaw`, `residual`,
    `deviation` (per motor) and `spread_pct`.

    `residual` is the part of the standing deviation that no control axis
    explains. A large residual on a quad means the numbers are not a simple
    trim — suspect a failing motor or a bad RPM channel rather than geometry.
    """
    means = np.asarray(means, dtype=float)
    if means.size != mix.n:
        raise ValueError(f"{means.size} motors given, mix expects {mix.n}")
    dev = means - means.mean()
    out = {"throttle": float(means.mean()), "deviation": dev}
    explained = np.zeros_like(dev)
    for name, f in mix.axes().items():
        denom = float(np.dot(f, f))
        coef = float(np.dot(dev, f) / denom) if denom else 0.0
        out[name] = 2.0 * coef              # full-scale swing, see module docstring
        explained += coef * f
    out["residual"] = float(np.sqrt(np.mean((dev - explained) ** 2)))
    out["spread_pct"] = float((means.max() - means.min()) / means.mean() * 100.0) if means.mean() else float("nan")
    return out
