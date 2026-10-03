"""Rate-loop simulator: an AC_PID replica, a low-order plant, the closed loop, and the
multicopter AutoTune twitch procedure run in software.

This is a **test oracle and the tier-B engine of `alog tune`, not a flight simulator**:
one axis at a time, no mixer, no motor saturation beyond clipping the controller output to
±1, no coupling between axes, no gravity, no wind. Its job is to give the analysis code a
plant whose truth is known - the true frequency response, the true closed-loop step
response, and the gains ArduCopter's AutoTune would find on it - so every tier can be
tested against that truth (docs/pid-tuning-plan.md §2.4, §6, §9 item 5).

Everything here is **deterministic**: no randomness anywhere. A caller that wants
measurement noise passes an array it generated itself (tests/tunesynth.py does).

What is replicated, and from where (reference/pid-tuning-sources.md):

* `ACPID` - `AC_PID::update_all` (§2): FLTT on the target, FLTE on the error, FLTD on the
  derivative of the filtered error, first-order alphas from `calc_lowpass_alpha_dt`, the
  integrator with its IMAX clamp and the anti-windup rule, the `AP_PIDInfo` fields the
  `PIDx` log message carries. SMAX is modelled as 0 (`Dmod` = 1.0 always) but `SRate` is
  computed by a simplified SlewLimiter - see `SlewRate`.
* `Plant` - `k e^{-τd s} / ((τ1 s + 1)(τ2 s + 1))`, discretised zero-order-hold at the loop
  rate with `scipy.signal.cont2discrete`; the delay is an integer sample buffer.
* `ClosedLoop` - the rate loop and, above it, the angle loop with ArduPilot's
  `sqrt_controller` (§3), angle = ∫ rate.
* `autotune()` - `AC_AutoTune_Multi` (§1.3-§1.6) with the firmware's constants: targets
  from `max_rate_step_bf_*`, the RATE_D_UP → RATE_D_DOWN → RATE_P_UP → ANGLE_P_DOWN →
  ANGLE_P_UP sequence, the twitch measurement (`twitching_test_rate/angle`, the early-stop
  timeout, the abort), the gain update rules with their limits, `SUCCESS_COUNT` 4 with
  `ignore_next`, the acceleration measurement, the GMBK backoffs and the saved I/P ratios.

Known simplifications (each stated where it applies): the aircraft is reset to level and
still between twitches instead of flying back to level with intra-test gains; the
firmware's millisecond integer clock is emulated from the loop count; the slew-rate
estimator is a monotone simplification of `Filter/SlewLimiter.cpp`.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

try:
    from scipy.signal import cont2discrete
except ImportError:                      # pragma: no cover - scipy is a dependency
    cont2discrete = None

__all__ = ["calc_lowpass_alpha_dt", "sqrt_controller", "LowPass", "SlewRate", "ACPID",
           "Plant", "ClosedLoop", "autotune", "AUTOTUNE", "GAIN_FIELDS", "gain_dict",
           "max_rate_step_bf", "TUNE_STEPS"]

# The names WP1's GainSet carries; `gain_dict()` accepts a dict, a dataclass or any
# object with these attributes, so a GainSet can be passed straight through.
GAIN_FIELDS = ("rat_p", "rat_i", "rat_d", "rat_ff", "fltd", "fltt", "flte", "smax", "imax",
               "ang_p", "acc_max_dps2", "ff_enab", "gyro_filter", "thst_hover", "loop_hz",
               "aggr", "gmbk", "min_d")

_GAIN_DEFAULTS = dict(rat_p=0.135, rat_i=0.135, rat_d=0.0036, rat_ff=0.0, fltd=20.0, fltt=20.0,
                      flte=0.0, smax=0.0, imax=0.5, ang_p=4.5, acc_max_dps2=1100.0, ff_enab=True,
                      gyro_filter=40.0, thst_hover=None, loop_hz=400.0, aggr=0.075, gmbk=0.25,
                      min_d=0.0005)


def gain_dict(gains):
    """A plain dict with every GAIN_FIELDS key, from a dict / dataclass / object.

    Missing keys take the AC_AttitudeControl_Multi defaults (sources §2, §3). Extra keys
    (`axis`, `defaulted`, `param_names` on a GainSet) are ignored.
    """
    if gains is None:
        src = {}
    elif isinstance(gains, dict):
        src = gains
    elif hasattr(gains, "__dataclass_fields__"):
        import dataclasses
        src = dataclasses.asdict(gains)
    else:
        src = {k: getattr(gains, k) for k in GAIN_FIELDS if hasattr(gains, k)}
    out = dict(_GAIN_DEFAULTS)
    for k in GAIN_FIELDS:
        if k in src and src[k] is not None:
            out[k] = src[k]
    return out


# ------------------------------------------------------------------ ArduPilot primitives

def calc_lowpass_alpha_dt(dt, cutoff_hz):
    """`AP_Math::calc_lowpass_alpha_dt`: first-order filter coefficient.

    0 Hz (or a non-positive dt) means the filter is off and the alpha is 1.0.
    """
    if dt < 0 or cutoff_hz < 0:
        return 1.0
    if cutoff_hz == 0:
        return 1.0
    if dt == 0:
        return 0.0
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    return min(max(dt / (dt + rc), 0.0), 1.0)


def sqrt_controller(error, p, second_ord_lim, dt):
    """`AC_AttitudeControl::sqrt_controller` (sources §3): proportional near zero, square
    root beyond `linear_dist = a / p²`, clamped to `|error| / dt`."""
    if second_ord_lim <= 0.0:
        correction = error * p
    elif p == 0.0:
        if error > 0:
            correction = math.sqrt(2.0 * second_ord_lim * error)
        elif error < 0:
            correction = -math.sqrt(2.0 * second_ord_lim * -error)
        else:
            correction = 0.0
    else:
        linear_dist = second_ord_lim / (p * p)
        if error > linear_dist:
            correction = math.sqrt(2.0 * second_ord_lim * (error - linear_dist / 2.0))
        elif error < -linear_dist:
            correction = -math.sqrt(2.0 * second_ord_lim * (-error - linear_dist / 2.0))
        else:
            correction = error * p
    if dt:
        lim = abs(error) / dt
        correction = min(max(correction, -lim), lim)
    return correction


class LowPass:
    """`LowPassFilterFloat`: `y += alpha (x - y)`."""

    __slots__ = ("hz", "y", "_init")

    def __init__(self, hz):
        self.hz = float(hz)
        self.reset(0.0)

    def reset(self, value=0.0):
        self.y = float(value)
        self._init = True

    def apply(self, sample, dt):
        if not self._init:
            self.y = sample
            self._init = True
            return self.y
        self.y += calc_lowpass_alpha_dt(dt, self.hz) * (sample - self.y)
        return self.y


class SlewRate:
    """A simplified `Filter/SlewLimiter.cpp` measuring the slew rate of P+D.

    The firmware: the sample's derivative through a 25 Hz low-pass; positive and negative
    peaks held for WINDOW_MS (300 ms) and then decayed; the mean of the two peaks; an
    attack filter (instant rise, slow fall) with time constant `tau` (1 s in AC_PID). The
    modifier (`Dmod`) engages only when SMAX > 0 and only after two consecutive
    exceedances; SMAX is 0 in every test gain set here, so `Dmod` is 1.0 and only the
    measurement is reproduced.

    Simplifications, all monotone in the ringing amplitude so a ringing loop reads a
    larger `SRate` than a quiet one: no exceedance-event bookkeeping (it only affects the
    modifier), the peak decay uses the same 25 Hz alpha as the derivative filter, and the
    attack filter rises instantly. Units: (normalised output) / s, as `PIDx.SRate`.
    """

    WINDOW_S = 0.300
    CUTOFF_HZ = 25.0

    def __init__(self, tau=1.0):
        self.tau = float(tau)
        self.reset()

    def reset(self):
        self.last_sample = 0.0
        self.filt = 0.0
        self.max_pos = self.max_neg = 0.0
        self.t_pos = self.t_neg = 0.0
        self.t = 0.0
        self.output = 0.0

    def update(self, sample, dt):
        if dt <= 0:
            return self.output
        self.t += dt
        alpha = calc_lowpass_alpha_dt(dt, self.CUTOFF_HZ)
        self.filt += alpha * ((sample - self.last_sample) / dt - self.filt)
        self.last_sample = sample
        slew = self.filt
        if slew > self.max_pos:
            self.max_pos, self.t_pos = slew, self.t
        elif self.t - self.t_pos > self.WINDOW_S:
            self.max_pos *= (1.0 - alpha)
        if -slew > self.max_neg:
            self.max_neg, self.t_neg = -slew, self.t
        elif self.t - self.t_neg > self.WINDOW_S:
            self.max_neg *= (1.0 - alpha)
        raw = 0.5 * (self.max_pos + self.max_neg)
        if raw >= self.output:
            self.output = raw
        else:
            self.output += (raw - self.output) * dt / (dt + self.tau)
        return self.output


# ------------------------------------------------------------------------------ AC_PID

class ACPID:
    """A replica of `AC_PID::update_all` (sources §2).

    Gains are attributes (`kp, ki, kd, kff, kdff, imax, fltt, flte, fltd`) so a caller can
    change them between calls exactly as `load_gains()` does in the firmware, without
    resetting the filters or the integrator. Units are the caller's; ArduCopter's rate
    PIDs are fed **rad/s** (`ClosedLoop` and `autotune` convert, and report the logged
    `Tar/Act/Err` in deg/s as the firmware's log messages do). After each `update_all` the `AP_PIDInfo`
    fields are on the object: `target` (post-FLTT), `actual`, `error` (post-FLTE), `P`,
    `I`, `D`, `FF`, `DFF`, `Dmod` (always 1.0: SMAX modelled as 0), `slew_rate`, `limit`
    and `flags` (bit0 LIMIT, bit2 RESET).
    """

    def __init__(self, kp, ki, kd, kff=0.0, imax=0.5, fltt=20.0, flte=0.0, fltd=20.0, kdff=0.0):
        self.kp, self.ki, self.kd, self.kff, self.kdff = float(kp), float(ki), float(kd), float(kff), float(kdff)
        self.imax = float(imax)
        self.fltt, self.flte, self.fltd = float(fltt), float(flte), float(fltd)
        self.slew = SlewRate()
        self.reset()

    # AC_PID keeps its alphas as functions of dt; the names match the firmware's
    def get_filt_T_alpha(self, dt):
        return calc_lowpass_alpha_dt(dt, self.fltt)

    def get_filt_E_alpha(self, dt):
        return calc_lowpass_alpha_dt(dt, self.flte)

    def get_filt_D_alpha(self, dt):
        return calc_lowpass_alpha_dt(dt, self.fltd)

    def reset(self):
        """`reset_filter()` + `reset_I()`: the next update seeds the filters from its
        inputs and reports the RESET flag."""
        self._reset_filter = True
        self._target = self._error = self._derivative = self._target_derivative = 0.0
        self._integrator = 0.0
        self.target = self.actual = self.error = 0.0
        self.P = self.I = self.D = self.FF = self.DFF = 0.0
        self.Dmod, self.slew_rate, self.limit, self.flags = 1.0, 0.0, False, 0
        self.slew.reset()

    def reset_I(self):
        self._integrator = 0.0

    def set_gains(self, **kw):
        for k, v in kw.items():
            if not hasattr(self, k):
                raise AttributeError(f"ACPID has no gain {k!r}")
            setattr(self, k, float(v))

    def update_all(self, target, measurement, dt, limit=False):
        """One controller step. Returns P + D + I; FF and DFF are on the object (the
        attitude controller adds them itself, and so does `ClosedLoop`)."""
        reset = False
        if self._reset_filter:
            self._reset_filter = False
            reset = True
            self._target = target
            self._error = self._target - measurement
            self._derivative = 0.0
            self._target_derivative = 0.0
        else:
            error_last = self._error
            target_last = self._target
            self._target += self.get_filt_T_alpha(dt) * (target - self._target)
            self._error += self.get_filt_E_alpha(dt) * ((self._target - measurement) - self._error)
            if dt > 0:
                derivative = (self._error - error_last) / dt
                self._derivative += self.get_filt_D_alpha(dt) * (derivative - self._derivative)
                self._target_derivative = (self._target - target_last) / dt
        # update_i(): no integration while the output is saturated and the error would
        # drive the integrator further into the limit
        if self.ki != 0.0 and dt > 0:
            if (not limit) or (self._integrator > 0 and self._error < 0) or (self._integrator < 0 and self._error > 0):
                self._integrator += self._error * self.ki * dt
                self._integrator = min(max(self._integrator, -self.imax), self.imax)
        else:
            self._integrator = 0.0
        P_out = self._error * self.kp
        D_out = self._derivative * self.kd
        # The slew limiter sees the *previous* P + D, as in the firmware
        self.slew_rate = self.slew.update(self.P + self.D, dt)
        self.Dmod = 1.0
        self.target, self.actual, self.error = self._target, measurement, self._error
        self.P, self.D, self.I = P_out, D_out, self._integrator
        self.FF = self._target * self.kff
        self.DFF = self._target_derivative * self.kdff
        self.limit = bool(limit)
        self.flags = (1 if limit else 0) | (4 if reset else 0)
        return P_out + D_out + self._integrator


# ------------------------------------------------------------------------------- Plant

class Plant:
    """`G(s) = k e^{-delay s} / ((tau1 s + 1)(tau2 s + 1))`, output → body rate.

    `k` is the steady rate (deg/s) per unit of normalised controller output; `tau1`,
    `tau2` seconds; `delay` seconds. The rational part is discretised zero-order-hold at
    `loop_hz` with `scipy.signal.cont2discrete`; the delay is `round(delay * loop_hz)`
    whole samples (so at 400 Hz it is quantised to 2.5 ms and the quantised value is on
    `delay_samples`; `delay_s_effective` is what the discrete model actually applies).
    `tau2 = 0` gives a single pole.
    """

    def __init__(self, k, tau1, tau2=0.0, delay=0.0, loop_hz=400.0):
        self.k, self.tau1, self.tau2, self.delay = float(k), float(tau1), float(tau2), float(delay)
        self.loop_hz = float(loop_hz)
        self.dt = 1.0 / self.loop_hz
        if self.tau1 <= 0:
            raise ValueError("tau1 must be positive")
        if cont2discrete is None:
            raise RuntimeError("scipy is required for Plant")
        if self.tau2 > 0:
            A = np.array([[-1.0 / self.tau1, 0.0], [1.0 / self.tau2, -1.0 / self.tau2]])
            B = np.array([[self.k / self.tau1], [0.0]])
            C = np.array([[0.0, 1.0]])
        else:
            A = np.array([[-1.0 / self.tau1]])
            B = np.array([[self.k / self.tau1]])
            C = np.array([[1.0]])
        D = np.array([[0.0]])
        Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), self.dt, method="zoh")
        self.Ad, self.Bd, self.Cd = Ad, Bd.ravel(), Cd.ravel()
        self.delay_samples = int(round(self.delay * self.loop_hz))
        self.delay_s_effective = self.delay_samples / self.loop_hz
        self.reset()

    def copy(self):
        return Plant(self.k, self.tau1, self.tau2, self.delay, self.loop_hz)

    def params(self):
        return dict(k=self.k, tau1=self.tau1, tau2=self.tau2, delay=self.delay, loop_hz=self.loop_hz,
                    delay_samples=self.delay_samples)

    def reset(self):
        self.x = np.zeros(self.Ad.shape[0])
        self._buf = deque([0.0] * self.delay_samples, maxlen=max(self.delay_samples, 1))
        self.y = 0.0

    def step(self, u):
        """Apply output `u` for one loop; returns the rate (deg/s) measured at the end of
        the loop, i.e. the gyro sample the next controller call sees."""
        if self.delay_samples:
            u_eff = self._buf[0]              # the input from delay_samples loops ago
            self._buf.append(float(u))
        else:
            u_eff = float(u)
        self.x = self.Ad @ self.x + self.Bd * u_eff
        self.y = float(self.Cd @ self.x)
        return self.y

    def freq_response(self, freqs_hz):
        """Continuous-time `G(j2πf)` (complex array) - the truth an identification is
        measured against. Uses the nominal delay, not the quantised one."""
        w = 2.0 * np.pi * np.asarray(freqs_hz, dtype=float)
        s = 1j * w
        G = self.k * np.exp(-s * self.delay) / ((self.tau1 * s + 1.0) * ((self.tau2 * s + 1.0) if self.tau2 > 0 else 1.0))
        return G

    def true_closed_loop_step(self, gains, seconds=0.5, amplitude=1.0, angle_loop=False):
        """The closed loop's response to a rate (or, with `angle_loop`, angle) step of
        `amplitude` under `gains`: (t, act) arrays. The plant is copied, so the caller's
        instance keeps its state."""
        loop = ClosedLoop(self.copy(), gains, self.loop_hz)
        n = int(round(seconds * self.loop_hz))
        cmd = np.full(n, float(amplitude))
        res = loop.run(n, angle_cmd=cmd) if angle_loop else loop.run(n, rate_cmd=cmd)
        return res["t"], res["act"]


# -------------------------------------------------------------------------- ClosedLoop

class ClosedLoop:
    """One axis: the angle loop feeding the rate loop feeding the plant.

    `gains` is anything `gain_dict()` accepts. `sqrt` selects ArduPilot's sqrt controller
    for the angle-error → rate mapping (`False` = plain `ang_p * error`, what AutoTune's
    test gains use). `shape` enables the 4.x pilot-input shaping (`input_shaping_angle`:
    sqrt controller with `p = 1/input_tc` and the ACC_MAX limit, then an
    acceleration-limited feed-forward rate) so a commanded angle step becomes the shaped
    target a real log shows in `ANG.DesRoll`; without it the angle command is the target.
    `ff` adds the shaped feed-forward rate to the rate target (`ATC_RATE_FF_ENAB`).

    The rate controller's output is clipped to ±1 and the `limit` flag passed back to
    the PID exactly as the mixer's saturation flag would be.
    """

    def __init__(self, plant, gains, loop_hz=None, sqrt=True, shape=True, ff=None, input_tc=0.15):
        self.plant = plant
        self.g = gain_dict(gains)
        self.loop_hz = float(loop_hz or plant.loop_hz)
        if abs(self.loop_hz - plant.loop_hz) > 1e-9:
            raise ValueError(f"loop_hz {self.loop_hz} differs from the plant's {plant.loop_hz}")
        self.dt = 1.0 / self.loop_hz
        self.sqrt, self.shape = bool(sqrt), bool(shape)
        self.ff = bool(self.g["ff_enab"]) if ff is None else bool(ff)
        self.input_tc = float(input_tc)
        g = self.g
        self.pid = ACPID(g["rat_p"], g["rat_i"], g["rat_d"], kff=g["rat_ff"], imax=g["imax"],
                         fltt=g["fltt"], flte=g["flte"], fltd=g["fltd"])
        self.reset()

    def reset(self):
        self.plant.reset()
        self.pid.reset()
        self.angle = 0.0            # deg, integrated true rate
        self.rate = 0.0             # deg/s, the plant's true rate (last sample)
        self.angle_target = 0.0     # deg, shaped attitude target
        self.rate_ff = 0.0          # deg/s, shaped feed-forward rate
        self._limit = False         # the mixer's saturation flag fed back to the PID

    def set_gains(self, gains):
        """Switch gains without resetting state (`load_gains()` / a PARM write)."""
        self.g = gain_dict(gains)
        g = self.g
        self.pid.set_gains(kp=g["rat_p"], ki=g["rat_i"], kd=g["rat_d"], kff=g["rat_ff"], imax=g["imax"],
                           fltt=g["fltt"], flte=g["flte"], fltd=g["fltd"])
        if self.ff is None:
            self.ff = bool(g["ff_enab"])

    def _accel_limit(self):
        a = self.g["acc_max_dps2"]
        return float(a) if a and a > 0 else 0.0

    def _shape(self, cmd):
        """4.x `input_euler_angle_roll_pitch_yaw` with rate feed-forward enabled."""
        dt = self.dt
        self.angle_target += self.rate_ff * dt
        accel = self._accel_limit()
        err = cmd - self.angle_target
        desired = sqrt_controller(err, 1.0 / max(self.input_tc, 0.01), accel, dt)
        if accel > 0:
            lo, hi = self.rate_ff - accel * dt, self.rate_ff + accel * dt
            desired = min(max(desired, lo), hi)
        self.rate_ff = desired

    def rate_target_from_angle(self, angle_cmd):
        """The rate the angle loop asks for, given the pilot's (or AutoTune's) angle."""
        if self.shape:
            self._shape(angle_cmd)
        else:
            self.angle_target, self.rate_ff = float(angle_cmd), 0.0
        err = self.angle_target - self.angle
        if self.sqrt:
            accel = self._accel_limit()
            lim = min(max(accel / 2.0, 40.0), 720.0) if accel > 0 else 0.0
            r = sqrt_controller(err, self.g["ang_p"], lim, self.dt)
        else:
            r = err * self.g["ang_p"]
        return r + (self.rate_ff if self.ff else 0.0)

    def step(self, rate_target, measurement):
        """One loop: controller on (rate_target, measurement), then the plant.

        Returns the clipped output; the new true rate is on `self.rate` and the PID
        info on `self.pid`. `measurement` is what the gyro reads (the caller may add
        noise to the plant's true rate)."""
        pid = self.pid
        # ArduCopter's rate PIDs work in rad/s (that is what makes P = 0.135 a sensible
        # gain and why max_rate_step_bf_* returns rad/s); the log fields are deg/s
        pid.update_all(math.radians(rate_target), math.radians(measurement), self.dt, limit=self._limit)
        raw = pid.P + pid.I + pid.D + pid.FF + pid.DFF
        out = min(max(raw, -1.0), 1.0)
        self._limit = out != raw
        self.rate = self.plant.step(out)
        self.angle += self.rate * self.dt
        return out

    _limit = False

    def run(self, n_or_t, angle_cmd=None, rate_cmd=None, noise=None):
        """Simulate `n` loops (or `t` seconds) with a commanded angle series (deg) or a
        commanded rate series (deg/s), exactly one of them. `noise` (deg/s, same length)
        is added to the measurement fed back to the controller and recorded as `act`.

        Returns a dict of arrays: `t, rdes, tar, err, act, out, p, i, d, ff, dff, dmod,
        srate, flags, angle, des_angle, rate_ff, true_rate` - `tar` is the FLTT-filtered
        target as `PIDx.Tar` (deg/s), `err` the FLTE-filtered error (deg/s), `rdes` the
        pre-filter rate target as `RATE.xDes`, `act` the measurement as
        `PIDx.Act`/`RATE.x`, `out` the clipped output as `RATE.xOut`.
        """
        if (angle_cmd is None) == (rate_cmd is None):
            raise ValueError("pass exactly one of angle_cmd or rate_cmd")
        n = int(n_or_t) if isinstance(n_or_t, (int, np.integer)) else int(round(float(n_or_t) * self.loop_hz))
        cmd = np.asarray(angle_cmd if angle_cmd is not None else rate_cmd, dtype=float)
        if cmd.shape[0] < n:
            raise ValueError(f"command has {cmd.shape[0]} samples, need {n}")
        if noise is not None:
            noise = np.asarray(noise, dtype=float)
            if noise.shape[0] < n:
                raise ValueError("noise shorter than the run")
        keys = ("rdes", "tar", "err", "act", "out", "p", "i", "d", "ff", "dff", "dmod", "srate", "flags",
                "angle", "des_angle", "rate_ff", "true_rate")
        cols = {k: np.empty(n) for k in keys}
        for j in range(n):
            meas = self.rate + (noise[j] if noise is not None else 0.0)
            if angle_cmd is not None:
                rt = self.rate_target_from_angle(cmd[j])
            else:
                rt = cmd[j]
            out = self.step(rt, meas)
            pid = self.pid
            cols["rdes"][j] = rt
            cols["tar"][j] = math.degrees(pid.target)
            cols["err"][j] = math.degrees(pid.error)
            cols["act"][j] = meas
            cols["out"][j] = out
            cols["p"][j] = pid.P
            cols["i"][j] = pid.I
            cols["d"][j] = pid.D
            cols["ff"][j] = pid.FF
            cols["dff"][j] = pid.DFF
            cols["dmod"][j] = pid.Dmod
            cols["srate"][j] = pid.slew_rate
            cols["flags"][j] = pid.flags
            cols["angle"][j] = self.angle
            cols["des_angle"][j] = self.angle_target
            cols["rate_ff"][j] = self.rate_ff
            cols["true_rate"][j] = self.rate
        cols["t"] = np.arange(n) * self.dt
        cols["flags"] = cols["flags"].astype(int)
        return cols


# ---------------------------------------------------------------------------- AutoTune

# AC_AutoTune_Multi.cpp / AC_AutoTune.h constants (sources §1.2), quoted from master.
AUTOTUNE = dict(
    TESTING_STEP_TIMEOUT_S=2.0, RD_STEP=0.05, RP_STEP=0.05, SP_STEP=0.05,
    PI_RATIO_FOR_TESTING=0.1, PI_RATIO_FINAL=1.0, YAW_PI_RATIO_FINAL=0.1,
    RD_MAX=0.200, RLPF_MIN=1.0, RLPF_MAX=5.0, FLTE_MIN=2.5, RP_MIN=0.01, RP_MAX=2.0,
    SP_MAX=40.0, SP_MIN=0.5, RP_ACCEL_MIN=4000.0, Y_ACCEL_MIN=1000.0, Y_FILT_FREQ=10.0,
    D_UP_DOWN_MARGIN=0.2, ACCEL_RP_BACKOFF=1.0, ACCEL_Y_BACKOFF=1.0,
    TARGET_RATE_RLLPIT_CDS=18000.0, TARGET_MIN_RATE_RLLPIT_CDS=4500.0,
    TARGET_RATE_YAW_CDS=9000.0, TARGET_MIN_RATE_YAW_CDS=1500.0,
    TARGET_ANGLE_MAX_RP_SCALE=1.0 / 2.0, TARGET_ANGLE_MAX_Y_SCALE=1.0,
    TARGET_ANGLE_MIN_RP_SCALE=1.0 / 3.0, TARGET_ANGLE_MIN_Y_SCALE=1.0 / 6.0,
    ANGLE_ABORT_RP_SCALE=2.5 / 3.0, ANGLE_MAX_Y_SCALE=1.0, ANGLE_NEG_RP_SCALE=1.0 / 5.0,
    SUCCESS_COUNT=4, TEST_PI_RATIO=0.01,           # load_test_gains: I = 0.01 P
    RATE_RP_CONTROLLER_OUT_MAX=1.0, RATE_YAW_CONTROLLER_OUT_MAX=1.0,
)

# TuneType enum values as ATUN.TuneStep logs them
TUNE_STEPS = {"RATE_D_UP": 0, "RATE_D_DOWN": 1, "RATE_P_UP": 2, "RATE_FF_UP": 3, "ANGLE_P_DOWN": 4,
              "ANGLE_P_UP": 5, "MAX_GAINS": 6, "TUNE_CHECK": 7, "TUNE_COMPLETE": 8}
SEQUENCE = ("RATE_D_UP", "RATE_D_DOWN", "RATE_P_UP", "ANGLE_P_DOWN", "ANGLE_P_UP")
AXIS_ID = {"roll": 0, "pitch": 1, "yaw": 2, "yaw_d": 3}


def max_rate_step_bf(kp, kd, flte, fltd, dt, thst_hover, out_max=1.0):
    """`AC_AttitudeControl::max_rate_step_bf_roll` (sources §1.4): the rate step (rad/s)
    that saturates the rate controller in about four loops."""
    alpha = min(calc_lowpass_alpha_dt(dt, flte), calc_lowpass_alpha_dt(dt, fltd))
    rem = 1.0 - alpha
    th = min(max(thst_hover, 0.1), 0.5)
    return 2.0 * th * out_max / ((rem * rem * rem * alpha * kd) / dt + kp)


class _Tune:
    """The AutoTune state for one axis. Fields mirror the firmware's members."""

    def __init__(self, axis, g, aggr, gmbk, min_d, angle_max_deg, loop_hz):
        self.axis = axis
        self.is_yaw = axis == "yaw"
        self.g = g
        self.aggr = min(max(aggr, 0.05), 0.2)                  # AUTOTUNE_AGGR is constrained 0.05-0.2
        self.gmbk = None if gmbk is None else min(max(gmbk, 0.0), 0.5)
        self.min_d = min_d
        self.angle_max_cd = angle_max_deg * 100.0
        self.dt = 1.0 / loop_hz
        self.loop_hz = loop_hz
        # init_gains(): rp/sp from the aircraft, rd floored at MIN_D
        self.rp = float(g["rat_p"])
        self.rd = max(float(g["rat_d"]), min_d)
        self.sp = float(g["ang_p"])
        self.rlpf = float(g["flte"]) if float(g["flte"]) > 0 else AUTOTUNE["FLTE_MIN"]
        self.accel_cdss = float(g["acc_max_dps2"] or 0.0) * 100.0
        self.thst_hover = g["thst_hover"]
        self.success_counter = 0
        self.ignore_next = False
        self.step_scaler = 1.0
        self.positive_direction = False
        self.test_accel_max_cdss = 0.0
        self.tune_type = 0
        self.events = []
        self.messages = []
        self.failed = None

    # ---- test gains, as load_test_gains()
    def test_gains(self):
        g = self.g
        if self.is_yaw:
            return dict(kp=self.rp, ki=self.rp * AUTOTUNE["TEST_PI_RATIO"], kd=0.0, kff=0.0,
                        fltt=0.0, flte=self.rlpf, fltd=g["fltd"], imax=g["imax"])
        return dict(kp=self.rp, ki=self.rp * AUTOTUNE["TEST_PI_RATIO"], kd=self.rd, kff=0.0,
                    fltt=0.0, flte=g["flte"], fltd=g["fltd"], imax=g["imax"])

    # ---- test_init(): the targets for this twitch, from the loaded test gains
    def targets(self):
        tg = self.test_gains()
        rate_max = max_rate_step_bf(tg["kp"], tg["kd"], tg["flte"], tg["fltd"], self.dt, self.thst_hover)
        if self.is_yaw:
            angle_abort = self.angle_max_cd * AUTOTUNE["ANGLE_MAX_Y_SCALE"]
            tmax = max(AUTOTUNE["TARGET_MIN_RATE_YAW_CDS"], self.step_scaler * AUTOTUNE["TARGET_RATE_YAW_CDS"])
            target_rate = min(max(math.degrees(rate_max * 0.75) * 100.0, AUTOTUNE["TARGET_MIN_RATE_YAW_CDS"]), tmax)
            amin = self.angle_max_cd * AUTOTUNE["TARGET_ANGLE_MIN_Y_SCALE"]
            amax = self.angle_max_cd * AUTOTUNE["TARGET_ANGLE_MAX_Y_SCALE"]
        else:
            angle_abort = self.angle_max_cd * AUTOTUNE["TARGET_ANGLE_MAX_RP_SCALE"]
            tmax = max(AUTOTUNE["TARGET_MIN_RATE_RLLPIT_CDS"], self.step_scaler * AUTOTUNE["TARGET_RATE_RLLPIT_CDS"])
            target_rate = min(max(math.degrees(rate_max) * 100.0, AUTOTUNE["TARGET_MIN_RATE_RLLPIT_CDS"]), tmax)
            amin = self.angle_max_cd * AUTOTUNE["TARGET_ANGLE_MIN_RP_SCALE"]
            amax = self.angle_max_cd * AUTOTUNE["TARGET_ANGLE_MAX_RP_SCALE"]
        # max_angle_step_bf_* = max_rate_step / angle P (inferred; sources §1.4)
        angle_step = math.degrees(rate_max) * 100.0 / self.sp if self.sp > 0 else amax
        target_angle = min(max(angle_step, amin), amax)
        return target_rate, target_angle, angle_abort


def _twitch(tune, plant, step_name, keep=False):
    """One twitch: EXECUTING_TEST from a level, still start until the measurement
    logic says UPDATE_GAINS or ABORT. Returns the record the firmware would log plus
    the raw measurements the update rule needs.

    Simplification: the aircraft is reset to level and still (WAITING_FOR_LEVEL with
    intra-test gains is not flown), so `start_rate` = 0 and `start_angle` = 0 and the
    PID's integrator starts empty.
    """
    dt = tune.dt
    aggr = tune.aggr
    angle_test = step_name in ("ANGLE_P_DOWN", "ANGLE_P_UP")
    tg = tune.test_gains()
    pid = ACPID(tg["kp"], tg["ki"], tg["kd"], kff=0.0, imax=tg["imax"], fltt=tg["fltt"], flte=tg["flte"], fltd=tg["fltd"])
    plant.reset()
    target_rate, target_angle, angle_abort = tune.targets()
    dir_sign = 1.0 if tune.positive_direction else -1.0
    filt = LowPass(AUTOTUNE["Y_FILT_FREQ"] if tune.is_yaw else 2.0 * tg["fltd"])
    filt.reset(0.0)
    # test_init()
    test_rate_max = test_rate_min = 0.0
    test_angle_max = test_angle_min = 0.0
    accel_measure_rate_max = 0.0
    step_timeout_ms = int(AUTOTUNE["TESTING_STEP_TIMEOUT_S"] * 1000)
    angle_lim_neg = tune.angle_max_cd * AUTOTUNE["ANGLE_NEG_RP_SCALE"]
    angle_lim_max = tune.angle_max_cd * AUTOTUNE["ANGLE_ABORT_RP_SCALE"]
    angle = rate = 0.0
    limit = False
    outcome = None
    n = 0
    angle_target_cd = dir_sign * target_angle
    trace_angle, trace_rate = [], []
    trace_pid = [] if keep else None
    while True:
        # command (test_run): a held body-rate step, or the angle-P loop with sqrt off
        if angle_test:
            rate_target = tune.sp * (angle_target_cd - angle * 100.0) * 0.01
        else:
            rate_target = dir_sign * target_rate * 0.01
        pid.update_all(math.radians(rate_target), math.radians(rate), dt, limit=limit)   # rad/s inside
        raw = pid.P + pid.I + pid.D
        out = min(max(raw, -1.0), 1.0)
        limit = out != raw
        rate = plant.step(out)
        angle += rate * dt
        n += 1
        elapsed_ms = int(n * 1000.0 / tune.loop_hz + 0.5)     # the firmware's ms clock
        lean_angle = dir_sign * angle * 100.0                  # cdeg from start_angle (0)
        rotation_rate = filt.apply(dir_sign * rate * 100.0, dt)  # cdeg/s, start_rate 0
        trace_angle.append(lean_angle * 0.01)
        trace_rate.append(rotation_rate * 0.01)
        if keep:
            trace_pid.append((rate_target, math.degrees(pid.target), rate, math.degrees(pid.error), pid.P, pid.I,
                              pid.D, pid.FF, pid.DFF, pid.Dmod, pid.slew_rate, pid.flags, out, angle))
        if angle_test:
            # twitching_test_angle(lean_angle, rotation_rate, target_angle*(1+0.5*aggr), ...)
            atm = target_angle * (1.0 + 0.5 * aggr)
            if lean_angle > test_angle_max:
                test_angle_max = lean_angle
                test_angle_min = lean_angle
            if lean_angle < test_angle_min and test_angle_max > atm * 0.25:
                test_angle_min = lean_angle
            if rotation_rate > test_rate_max:
                test_rate_max = rotation_rate
                test_rate_min = rotation_rate
            if rotation_rate < test_rate_min:
                test_rate_min = rotation_rate
            if test_angle_max < atm * 0.6321:
                step_timeout_ms = min(elapsed_ms * 3, int(AUTOTUNE["TESTING_STEP_TIMEOUT_S"] * 1000))
            if test_angle_max > atm:
                outcome = "UPDATE_GAINS"
            if test_angle_max - test_angle_min > test_angle_max * aggr:
                outcome = "UPDATE_GAINS"
            if elapsed_ms >= step_timeout_ms:
                outcome = "UPDATE_GAINS"
            # twitching_measure_acceleration
            if accel_measure_rate_max < rotation_rate:
                accel_measure_rate_max = rotation_rate
                if elapsed_ms > 0:
                    tune.test_accel_max_cdss = 1000.0 * accel_measure_rate_max / elapsed_ms
        else:
            # twitching_test_rate(lean_angle, rotation_rate, target_rate, ...)
            if rotation_rate > test_rate_max:
                test_rate_max = rotation_rate
                test_rate_min = rotation_rate
                test_angle_min = lean_angle
            if rotation_rate < test_rate_min and test_rate_max > target_rate * 0.25:
                test_rate_min = rotation_rate
                test_angle_min = lean_angle
            if test_rate_max < target_rate * 0.6321:
                step_timeout_ms = min(elapsed_ms * 3, int(AUTOTUNE["TESTING_STEP_TIMEOUT_S"] * 1000))
            if test_rate_max > target_rate:
                outcome = "UPDATE_GAINS"
            if test_rate_max - test_rate_min > test_rate_max * aggr:
                outcome = "UPDATE_GAINS"
            if elapsed_ms >= step_timeout_ms:
                outcome = "UPDATE_GAINS"
            # twitching_measure_acceleration
            if accel_measure_rate_max < rotation_rate:
                accel_measure_rate_max = rotation_rate
                if elapsed_ms > 0:
                    tune.test_accel_max_cdss = 1000.0 * accel_measure_rate_max / elapsed_ms
            # twitching_abort_rate(lean_angle, rotation_rate, angle_abort, test_rate_min, test_angle_min)
            if lean_angle >= angle_abort:
                if rotation_rate == test_rate_min or test_angle_min > 0.95 * angle_abort:
                    if tune.step_scaler > 0.2:
                        tune.step_scaler *= 0.9
                        outcome = "ABORT"
                    else:
                        tune.events.append(35)
                        tune.messages.append("AutoTune: Twitch Size Determination Failed")
                        tune.failed = "Twitch Size Determination Failed"
                        tune.events.append(34)
                        outcome = "ABORT"
                else:
                    outcome = "UPDATE_GAINS"
        # EXECUTING_TEST abort (AC_AutoTune::control_attitude): the test axis moved the
        # wrong way past ANGLE_MAX/5, or the roll/pitch lean exceeded 2.5/3 ANGLE_MAX
        # (the lean check is on the aircraft's lean angle, so it cannot fire on yaw here)
        if lean_angle <= -angle_lim_neg or (not tune.is_yaw and abs(angle) * 100.0 > angle_lim_max):
            outcome = "ABORT"
        if outcome is not None or n > 4 * tune.loop_hz:
            break
    if outcome is None:
        outcome = "UPDATE_GAINS"                  # cannot happen: the 2 s timeout fires first
    if angle_test:
        targ, mn, mx = target_angle, test_angle_min, test_angle_max
    else:
        targ, mn, mx = target_rate, test_rate_min, test_rate_max
    return dict(outcome=outcome, targ=targ, min=mn, max=mx, rate_min=test_rate_min, rate_max=test_rate_max,
                angle_max=test_angle_max, elapsed_s=n * dt, direction=int(dir_sign),
                trace_angle=np.array(trace_angle), trace_rate=np.array(trace_rate),
                trace_pid=np.array(trace_pid) if keep else None)


def _update(tune, step_name, m):
    """The UPDATE_GAINS rule for one step (sources §1.5, quoted from master). Returns
    True when success_counter was incremented by this twitch."""
    A = AUTOTUNE
    aggr = tune.aggr
    before = tune.success_counter
    if tune.is_yaw:
        # yaw(E): the "D" being searched is FLTE in [RLPF_MIN, RLPF_MAX]
        d_attr, d_min, d_max = "rlpf", A["RLPF_MIN"], A["RLPF_MAX"]
    else:
        d_attr, d_min, d_max = "rd", tune.min_d, A["RD_MAX"]
    rp_min, rp_max = A["RP_MIN"], A["RP_MAX"]
    rd_step, rp_step, sp_step = A["RD_STEP"], A["RP_STEP"], A["SP_STEP"]
    d = getattr(tune, d_attr)
    p = tune.rp
    if step_name == "RATE_D_UP":
        if m["max"] > m["targ"]:
            p -= p * rp_step
            if p < rp_min:
                p = rp_min
                d -= d * rd_step
                if d <= d_min:
                    d = d_min
                    tune.success_counter = A["SUCCESS_COUNT"]
                    tune.events.append(35)
                    tune.messages.append("AutoTune: Min Rate D limit reached")
        elif m["max"] < m["targ"] * (1.0 - A["D_UP_DOWN_MARGIN"]) and p <= rp_max:
            p += p * rp_step
            if p >= rp_max:
                p = rp_max
                tune.events.append(35)
        else:
            if m["max"] - m["min"] > m["max"] * aggr:
                tune.ignore_next = True
                tune.success_counter += 1
            else:
                if not tune.ignore_next:
                    if tune.success_counter > 0:
                        tune.success_counter -= 1
                    d += d * rd_step * 2.0
                    if d >= d_max:
                        d = d_max
                        tune.success_counter = A["SUCCESS_COUNT"]
                        tune.events.append(35)
                else:
                    tune.ignore_next = False
    elif step_name == "RATE_D_DOWN":
        if m["max"] > m["targ"]:
            p -= p * rp_step
            if p < rp_min:
                p = rp_min
                d -= d * rd_step
                if d <= d_min:
                    d = d_min
                    tune.success_counter = A["SUCCESS_COUNT"]
                    tune.events.append(35)
                    tune.messages.append("AutoTune: Min Rate D limit reached")
        elif m["max"] < m["targ"] * (1.0 - A["D_UP_DOWN_MARGIN"]) and p <= rp_max:
            p += p * rp_step
            if p >= rp_max:
                p = rp_max
                tune.events.append(35)
        else:
            if m["max"] - m["min"] < m["max"] * aggr:
                if not tune.ignore_next:
                    tune.success_counter += 1
                else:
                    tune.ignore_next = False
            else:
                tune.ignore_next = True
                if tune.success_counter > 0:
                    tune.success_counter -= 1
                d -= d * rd_step
                if d <= d_min:
                    d = d_min
                    tune.success_counter = A["SUCCESS_COUNT"]
                    tune.events.append(35)
                    tune.messages.append("AutoTune: Min Rate D limit reached")
    elif step_name == "RATE_P_UP":
        fail_min_d = not tune.is_yaw
        if m["max"] > m["targ"] * (1.0 + 0.5 * aggr):
            tune.ignore_next = True
            tune.success_counter += 1
        elif (m["max"] < m["targ"] and m["max"] > m["targ"] * (1.0 - A["D_UP_DOWN_MARGIN"])
              and m["max"] - m["min"] > m["max"] * aggr and d > d_min):
            if tune.success_counter > 0:
                tune.success_counter -= 1
            d -= d * rd_step
            if d <= d_min:
                d = d_min
                tune.events.append(35)
                if fail_min_d:
                    tune.messages.append("AutoTune: Rate D Gain Determination Failed")
                    tune.failed = "Rate D Gain Determination Failed"
                    tune.events.append(34)
            p -= p * rp_step
            if p <= rp_min:
                p = rp_min
                tune.messages.append("AutoTune: Rate P Gain Determination Failed")
                tune.failed = "Rate P Gain Determination Failed"
                tune.events.append(34)
        else:
            if not tune.ignore_next:
                if tune.success_counter > 0:
                    tune.success_counter -= 1
                p += p * rp_step
                if p >= rp_max:
                    p = rp_max
                    tune.success_counter = A["SUCCESS_COUNT"]
                    tune.events.append(35)
            else:
                tune.ignore_next = False
    elif step_name == "ANGLE_P_DOWN":
        sp = tune.sp
        if m["max"] < m["targ"] * (1.0 + 0.5 * aggr):
            if not tune.ignore_next:
                tune.success_counter += 1
            else:
                tune.ignore_next = False
        else:
            tune.ignore_next = True
            if tune.success_counter > 0:
                tune.success_counter -= 1
            sp -= sp * sp_step
            if sp <= A["SP_MIN"]:
                sp = A["SP_MIN"]
                tune.events.append(35)
                tune.messages.append("AutoTune: Angle P Gain Determination Failed")
                tune.failed = "Angle P Gain Determination Failed"
                tune.events.append(34)
        tune.sp = sp
    elif step_name == "ANGLE_P_UP":
        sp = tune.sp
        if (m["max"] > m["targ"] * (1.0 + 0.5 * aggr)
                or (m["max"] > m["targ"] and m["rate_min"] < -m["rate_max"] * aggr)):
            tune.ignore_next = True
            tune.success_counter += 1
        else:
            if not tune.ignore_next:
                if tune.success_counter > 0:
                    tune.success_counter -= 1
                sp += sp * sp_step
                if sp >= A["SP_MAX"]:
                    sp = A["SP_MAX"]
                    tune.success_counter = A["SUCCESS_COUNT"]
                    tune.events.append(35)
            else:
                tune.ignore_next = False
        tune.sp = sp
    else:
        raise ValueError(step_name)
    setattr(tune, d_attr, d)
    tune.rp = p
    return tune.success_counter > before


def _backoff(tune, step_name):
    """set_tuning_gains_with_backoff(): applied when a step completes."""
    gb = tune.gmbk if tune.gmbk is not None else 0.0
    if step_name == "RATE_P_UP":
        if tune.is_yaw:
            tune.rp *= (1.0 - gb)
        else:
            tune.rd *= (1.0 - gb)
            tune.rp *= (1.0 - gb)
    elif step_name == "ANGLE_P_UP":
        tune.sp *= (1.0 - gb) * (1.0 - tune.aggr)
        if tune.is_yaw:
            tune.accel_cdss = max(AUTOTUNE["Y_ACCEL_MIN"], tune.test_accel_max_cdss * AUTOTUNE["ACCEL_Y_BACKOFF"])
        else:
            tune.accel_cdss = max(AUTOTUNE["RP_ACCEL_MIN"], tune.test_accel_max_cdss * AUTOTUNE["ACCEL_RP_BACKOFF"])


def autotune(plant, gains, aggr=None, gmbk=None, min_d=None, axis="roll", angle_max_deg=45.0,
             loop_hz=None, max_twitches=400, keep_traces=False):
    """Run the multicopter AutoTune sequence on `plant` starting from `gains`.

    `gains` is anything `gain_dict()` accepts (a GainSet via `dataclasses.asdict`, or a
    dict with those names). `aggr`, `gmbk`, `min_d` default to the gain set's own
    (`AUTOTUNE_AGGR`, `AUTOTUNE_GMBK`, `AUTOTUNE_MIN_D`); `gmbk=None` in the gain set
    means no GMBK backoff (the 4.x behaviour is *not* emulated here - pass the branch's
    fixed backoff as `gmbk`, e.g. 0.0 for RD/RP_BACKOFF 1.0; see sources §1.6).
    `thst_hover` comes from the gain set (`CTUN.ThH` median or `MOT_THST_HOVER`); when it
    is None the parameter default 0.35 is used and `thst_hover_defaulted` is True.
    `angle_max_deg` is `ANGLE_MAX` (default 45°). `axis` is "roll", "pitch" or "yaw";
    yaw runs the yaw(E) variant that searches FLTE in [1, 5] Hz instead of D.

    The sequence is RATE_D_UP → RATE_D_DOWN → RATE_P_UP → ANGLE_P_DOWN → ANGLE_P_UP, each
    step ending after SUCCESS_COUNT (4) passes, then the GMBK backoffs and the save
    ratios (I = P; yaw I = 0.1 P). Direction alternates every twitch. The run stops with
    `aborted` set when the firmware would have declared FAILED, or after `max_twitches`.

    Returns a dict:
      rat_p, rat_i, rat_d, ang_p, acc_max_dps2, flte      the gains AutoTune would save
      twitches      one record per twitch, as ATUN logs it (Targ/Min/Max in deg or
                    deg/s, ddt in cdeg/s² unscaled) plus `axis`, `step`, `step_id`,
                    `direction`, `outcome`, `passed`, `success_counter`, `elapsed_s`
      steps_completed, gains_per_step (pre-backoff rp/rd/sp/rlpf when the step passed)
      final_overshoot (last RATE_P_UP twitch: max/targ − 1), final_bounce (last
                    RATE_D_DOWN twitch: (max − min)/max), test_accel_max_cdss
      messages      the GCS text lines the firmware would have sent, in order
      events        EV ids in order (35 REACHED_LIMIT, 34 FAILED, 33 SUCCESS)
      aborted       None, or the failure text
      params        the ATC_* values the save would write, keyed by 4.x name
      constants     the AUTOTUNE table, aggr/gmbk/min_d/thst_hover used
    """
    g = gain_dict(gains)
    aggr = float(g["aggr"] if aggr is None else aggr)
    gmbk = g["gmbk"] if gmbk is None else gmbk
    min_d = float(g["min_d"] if min_d is None else min_d)
    loop_hz = float(loop_hz or g["loop_hz"] or plant.loop_hz)
    if abs(loop_hz - plant.loop_hz) > 1e-9:
        raise ValueError(f"loop_hz {loop_hz} differs from the plant's {plant.loop_hz}")
    if axis not in ("roll", "pitch", "yaw"):
        raise ValueError(f"axis {axis!r}: expected roll, pitch or yaw")
    thst_defaulted = g["thst_hover"] is None
    if thst_defaulted:
        g["thst_hover"] = 0.35
    tune = _Tune(axis, g, aggr, gmbk, min_d, angle_max_deg, loop_hz)
    plant = plant.copy()
    twitches, steps_completed, gains_per_step = [], [], {}
    final_overshoot = final_bounce = None
    aborted = None
    tune.events.append(30)                                   # AUTOTUNE_INITIALISED
    for step_name in SEQUENCE:
        done = False
        while not done:
            if len(twitches) >= max_twitches:
                aborted = f"max_twitches {max_twitches} reached during {step_name}"
                break
            m = _twitch(tune, plant, step_name, keep=keep_traces)
            rec = dict(axis=axis, axis_id=AXIS_ID[axis], step=step_name, step_id=TUNE_STEPS[step_name],
                       targ=m["targ"] * 0.01, min=m["min"] * 0.01, max=m["max"] * 0.01,
                       rp=tune.rp, rd=tune.rlpf if tune.is_yaw else tune.rd, sp=tune.sp,
                       ddt=tune.test_accel_max_cdss, direction=m["direction"], outcome=m["outcome"],
                       elapsed_s=m["elapsed_s"], passed=False, success_counter=tune.success_counter)
            if keep_traces:
                # per-loop ATDE (angle, rate) and the PID columns: rdes, tar, act, err, P, I,
                # D, FF, DFF, Dmod, SRate, Flags, out, angle - what RATE/PIDx/ATDE would log
                rec["trace_angle"], rec["trace_rate"] = m["trace_angle"], m["trace_rate"]
                rec["trace_pid"] = m["trace_pid"]
            if m["outcome"] == "ABORT":
                twitches.append(rec)
                tune.positive_direction = not tune.positive_direction
                if tune.failed:
                    aborted = tune.failed
                    break
                continue
            # UPDATE_GAINS: ATUN is written with the gains *tested*, then the rule runs
            rec["passed"] = _update(tune, step_name, m)
            rec["success_counter"] = tune.success_counter
            twitches.append(rec)
            tune.positive_direction = not tune.positive_direction
            if step_name == "RATE_P_UP":
                final_overshoot = m["max"] / m["targ"] - 1.0 if m["targ"] else None
            if step_name == "RATE_D_DOWN":
                final_bounce = (m["max"] - m["min"]) / m["max"] if m["max"] else None
            if tune.failed:
                aborted = tune.failed
                break
            if tune.success_counter >= AUTOTUNE["SUCCESS_COUNT"]:
                tune.success_counter = 0
                tune.step_scaler = 1.0
                tune.ignore_next = False
                gains_per_step[step_name] = dict(rp=tune.rp, rd=tune.rd, sp=tune.sp, rlpf=tune.rlpf,
                                                 ddt=tune.test_accel_max_cdss)
                _backoff(tune, step_name)
                steps_completed.append(step_name)
                done = True
        if aborted:
            break
    complete = len(steps_completed) == len(SEQUENCE)
    if complete:
        tune.events.append(33)                               # AUTOTUNE_SUCCESS
    pi_ratio = AUTOTUNE["YAW_PI_RATIO_FINAL"] if tune.is_yaw else AUTOTUNE["PI_RATIO_FINAL"]
    rat_p = tune.rp
    rat_i = tune.rp * pi_ratio
    rat_d = float(g["rat_d"]) if tune.is_yaw else tune.rd
    acc = tune.accel_cdss / 100.0
    ax = {"roll": ("RLL", "R", "Roll"), "pitch": ("PIT", "P", "Pitch"), "yaw": ("YAW", "Y", "Yaw")}[axis]
    params = {f"ATC_RAT_{ax[0]}_P": rat_p, f"ATC_RAT_{ax[0]}_I": rat_i, f"ATC_RAT_{ax[0]}_D": rat_d,
              f"ATC_ANG_{ax[0]}_P": tune.sp, f"ATC_ACCEL_{ax[1]}_MAX": tune.accel_cdss}
    if tune.is_yaw:
        params[f"ATC_RAT_{ax[0]}_FLTE"] = tune.rlpf
    messages = list(tune.messages)
    if complete:
        messages.append(f"AutoTune: {ax[2]} Rate: P:{rat_p:0.3f}, I:{rat_i:0.3f}, D:{rat_d:0.4f}")
        messages.append(f"AutoTune: {ax[2]} Angle P:{tune.sp:0.3f}, Max Accel:{tune.accel_cdss:0.0f}")
    return dict(axis=axis, rat_p=rat_p, rat_i=rat_i, rat_d=rat_d, ang_p=tune.sp, acc_max_dps2=acc,
                flte=tune.rlpf if tune.is_yaw else float(g["flte"]),
                twitches=twitches, n_twitches=len(twitches), steps_completed=steps_completed,
                complete=complete, gains_per_step=gains_per_step, final_overshoot=final_overshoot,
                final_bounce=final_bounce, test_accel_max_cdss=tune.test_accel_max_cdss,
                messages=messages, events=list(tune.events), aborted=aborted, params=params,
                constants=dict(AUTOTUNE, aggr=tune.aggr, gmbk=tune.gmbk, min_d=min_d,
                               thst_hover=g["thst_hover"], thst_hover_defaulted=thst_defaulted,
                               angle_max_deg=angle_max_deg, loop_hz=loop_hz))
