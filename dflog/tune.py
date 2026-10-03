"""PID tuning from logs - the data model, signal extraction, segmentation and gates.

This is work package 1 of `docs/pid-tuning-plan.md`: the part every later tier (step
response, plant identification, AutoTune reconstruction, fusion) codes to. It turns a log
and a window into `AxisSignals` - one per axis per *parameter-constant* segment - or into
a structured `Refusal` that names what the log lacks and what to change. Nothing here
recommends a gain.

Rules this module follows (RULES.md 1-2):
  * A refusal is returned, never raised. The caller decides whether it is a SKIP, a WARN
    or an exit 3; the code, message and fix are the same in every rendering.
  * Every constant an algorithm uses is in `CONSTANTS` with its provenance; every graded
    number is in `checks.T`. Nothing numeric is hard-coded in a function body.
  * A parameter that was not in the log is defaulted from the firmware's own default and
    listed in `GainSet.defaulted`; which spelling of a drifting name was found is in
    `GainSet.param_names`.
  * Timing is measured, not assumed: `spectral.sample_rate()` gates every segment, so an
    irregular or 10 Hz stream is refused with the `LOG_BITMASK` that would fix it.

Units: deg/s for every rate; deg/s^2 for `acc_max_dps2` (converted from the cdeg/s^2
spelling); the plant input `out` is the normalised controller output (+-1).

Imports only numpy, pandas, the stdlib and `dflog.{parser,flight,spectral,stats,checks}`;
never `dflog.analysis` or `dflog.cli`.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field, asdict

import numpy as np

from .checks import T
from .spectral import SpectralError, sample_rate

__all__ = ["AXES", "PID_MSG", "RATE_COLS", "RAT_STEM", "ANG_P_PARAM", "ACC_MAX_PARAMS",
           "SEGMENT_PARAMS", "CONSTANTS", "DEFAULTS", "REFUSAL_CODES",
           "LOGGING_REQUIREMENTS", "LOGGING_FIX_HEADER",
           "GainSet", "AxisSignals", "StepResponse", "PlantModel", "Ceiling",
           "Recommendation", "TuneAnalysis", "Refusal",
           "segment", "extract_axes", "logging_requirements", "format_logging_fix",
           "logging_fix", "all_constants"]

# ----------------------------------------------------------------------- naming

AXES = ("roll", "pitch", "yaw")

#: Per-axis rate-PID log message (ArduCopter AC_PID logging; sources file section 2).
PID_MSG = {"roll": "PIDR", "pitch": "PIDP", "yaw": "PIDY"}

#: Per-axis RATE columns: (desired, actual, output). `RDes` is the rate-controller input
#: before FLTT, `R` the gyro the loop saw, `ROut` the normalised output incl. FF.
RATE_COLS = {"roll": ("RDes", "R", "ROut"), "pitch": ("PDes", "P", "POut"),
             "yaw": ("YDes", "Y", "YOut")}

#: Per-axis rate-PID parameter stem: `ATC_RAT_RLL_P` is RAT_STEM["roll"] + "P".
RAT_STEM = {"roll": "ATC_RAT_RLL_", "pitch": "ATC_RAT_PIT_", "yaw": "ATC_RAT_YAW_"}

#: Per-axis angle P.
ANG_P_PARAM = {"roll": "ATC_ANG_RLL_P", "pitch": "ATC_ANG_PIT_P", "yaw": "ATC_ANG_YAW_P"}

#: Per-axis acceleration limit, both spellings: (master deg/s^2, 4.x cdeg/s^2). The
#: sources file section 8: the Circuit log already lacks `ATC_ACCEL_R_MAX`, the Brisket
#: log has it at 116700 cdeg/s^2.
ACC_MAX_PARAMS = {"roll": ("ATC_ACC_R_MAX", "ATC_ACCEL_R_MAX"),
                  "pitch": ("ATC_ACC_P_MAX", "ATC_ACCEL_P_MAX"),
                  "yaw": ("ATC_ACC_Y_MAX", "ATC_ACCEL_Y_MAX")}

#: Parameters that change the rate loop on every axis: a change to one of these inside
#: the window splits the segment for every axis.
SEGMENT_PARAMS = ("INS_GYRO_FILTER", "ATC_RATE_FF_ENAB")

REFUSAL_CODES = ("NO_PID_MESSAGES", "PID_RATE_TOO_LOW", "IRREGULAR_SAMPLING",
                 "NO_EXCITATION", "OUTPUT_SATURATED", "GAINS_CHANGED_IN_FLIGHT",
                 "DIFFERENT_AIRCRAFT", "NO_COHERENCE", "AUTOTUNE_INCOMPLETE",
                 "WINDOW_TOO_SHORT")

# -------------------------------------------------------------------- constants

_SRC_AUTOTUNE = ("ArduCopter libraries/AC_AutoTune/AC_AutoTune_Multi.cpp (master 2026-09); "
                 "reference/pid-tuning-sources.md section 1.2")
_SRC_AUTOTUNE_H = "ArduCopter libraries/AC_AutoTune/AC_AutoTune.h; reference/pid-tuning-sources.md section 1.2"
_SRC_AUTOTUNE_PARAMS = ("ArduCopter AC_AutoTune_Multi var_info defaults; "
                        "reference/pid-tuning-sources.md section 1.1")
_SRC_AUTOTUNE_RULES = "AC_AutoTune_Multi.cpp update rules; reference/pid-tuning-sources.md sections 1.4-1.6"
_SRC_PIDA = ("PID-Analyzer (Florian Melsheimer, github.com/Plasmatree/PID-Analyzer) Trace class; "
             "reference/pid-tuning-sources.md section 7.2")
_SRC_QUIK = ("ArduPilot libraries/AP_Scripting/applets/VTOL-quicktune.lua defaults; "
             "reference/pid-tuning-sources.md section 4")
_SRC_PLAN = "docs/pid-tuning-plan.md section 2.3 (fpvpidlab-style minimum segment)"


def _c(value, source, note=""):
    return dict(value=value, source=source, note=note)


#: Algorithm constants (not gradings). Printed by `alog schema` as `tune_constants`.
CONSTANTS = {
    # --- PID-Analyzer / PIDReview step-response deconvolution ---------------------
    "step_frame_s":        _c(1.0, _SRC_PIDA, "length of each deconvolution frame, s"),
    "step_response_s":     _c(0.5, _SRC_PIDA, "length of the step response kept from each frame, s"),
    "step_overlap":        _c(16, _SRC_PIDA, "frames overlap by frame/16 (superposition ratio)"),
    "step_cut_hz":         _c(25.0, _SRC_PIDA, "Wiener regulariser cut-off: sn = 10 (1 - mask25Hz + 1e-9)"),
    "step_min_target_dps": _c(20.0, _SRC_PIDA, "frames whose max |target| is below this are dropped, deg/s"),
    "step_split_dps":      _c(500.0, _SRC_PIDA, "PID-Analyzer splits frames into low/high input at this |target|, deg/s"),

    # --- segmentation --------------------------------------------------------------
    "segment_min_s":       _c(10.0, _SRC_PLAN, "a parameter-constant, gap-free segment shorter than this is dropped, s"),

    # --- AutoTune (multicopter) parameters -----------------------------------------
    "autotune_aggr_default":  _c(0.075, _SRC_AUTOTUNE_PARAMS, "AUTOTUNE_AGGR default; bounce-back fraction"),
    "autotune_aggr_min":      _c(0.05, _SRC_AUTOTUNE_PARAMS, "AUTOTUNE_AGGR constrained low"),
    "autotune_aggr_max":      _c(0.2, _SRC_AUTOTUNE_PARAMS, "AUTOTUNE_AGGR constrained high (in code; wiki range 0.05-0.10)"),
    "autotune_gmbk_default":  _c(0.25, _SRC_AUTOTUNE_PARAMS, "AUTOTUNE_GMBK default (master); absent on 4.x, see sources 1.6"),
    "autotune_min_d_default": _c(0.0005, _SRC_AUTOTUNE_PARAMS, "AUTOTUNE_MIN_D default"),

    # --- AutoTune constants (quoted) -----------------------------------------------
    "autotune_testing_step_timeout_ms": _c(2000, _SRC_AUTOTUNE, "AUTOTUNE_TESTING_STEP_TIMEOUT_MS"),
    "autotune_rd_step":          _c(0.05, _SRC_AUTOTUNE, "AUTOTUNE_RD_STEP, multiplicative"),
    "autotune_rp_step":          _c(0.05, _SRC_AUTOTUNE, "AUTOTUNE_RP_STEP, multiplicative"),
    "autotune_sp_step":          _c(0.05, _SRC_AUTOTUNE, "AUTOTUNE_SP_STEP, multiplicative"),
    "autotune_rd_up_step":       _c(0.10, _SRC_AUTOTUNE_RULES, "RATE_D_UP raises D by 10 % when bounce-back is below AGGR"),
    "autotune_pi_ratio_testing": _c(0.1, _SRC_AUTOTUNE, "AUTOTUNE_PI_RATIO_FOR_TESTING: I = 0.1 P between tests"),
    "autotune_pi_ratio_test_gains": _c(0.01, _SRC_AUTOTUNE_RULES, "load_test_gains: I = 0.01 P during a twitch"),
    "autotune_pi_ratio_final":   _c(1.0, _SRC_AUTOTUNE, "AUTOTUNE_PI_RATIO_FINAL: I = P when saved (roll, pitch)"),
    "autotune_yaw_pi_ratio_final": _c(0.1, _SRC_AUTOTUNE, "AUTOTUNE_YAW_PI_RATIO_FINAL: I = 0.1 P when saved (yaw)"),
    "autotune_rd_max":           _c(0.200, _SRC_AUTOTUNE, "AUTOTUNE_RD_MAX"),
    "autotune_rp_min":           _c(0.01, _SRC_AUTOTUNE, "AUTOTUNE_RP_MIN"),
    "autotune_rp_max":           _c(2.0, _SRC_AUTOTUNE, "AUTOTUNE_RP_MAX"),
    "autotune_sp_min":           _c(0.5, _SRC_AUTOTUNE, "AUTOTUNE_SP_MIN"),
    "autotune_sp_max":           _c(40.0, _SRC_AUTOTUNE, "AUTOTUNE_SP_MAX"),
    "autotune_rlpf_min_hz":      _c(1.0, _SRC_AUTOTUNE, "AUTOTUNE_RLPF_MIN: yaw FLTE search floor"),
    "autotune_rlpf_max_hz":      _c(5.0, _SRC_AUTOTUNE, "AUTOTUNE_RLPF_MAX: yaw FLTE search ceiling"),
    "autotune_flte_min_hz":      _c(2.5, _SRC_AUTOTUNE, "AUTOTUNE_FLTE_MIN: yaw FLTE seed when it was 0"),
    "autotune_rp_accel_min_cdss": _c(4000, _SRC_AUTOTUNE, "AUTOTUNE_RP_ACCEL_MIN: floor for ATC_ACC_R/P_MAX, cdeg/s^2"),
    "autotune_y_accel_min_cdss": _c(1000, _SRC_AUTOTUNE, "AUTOTUNE_Y_ACCEL_MIN: floor for ATC_ACC_Y_MAX, cdeg/s^2"),
    "autotune_y_filt_freq_hz":   _c(10.0, _SRC_AUTOTUNE, "AUTOTUNE_Y_FILT_FREQ: gyro LPF while tuning yaw FLTE"),
    "autotune_d_up_down_margin": _c(0.2, _SRC_AUTOTUNE, "AUTOTUNE_D_UP_DOWN_MARGIN: peak must reach 80 % of target"),
    "autotune_accel_rp_backoff": _c(1.0, _SRC_AUTOTUNE, "AUTOTUNE_ACCEL_RP_BACKOFF"),
    "autotune_accel_y_backoff":  _c(1.0, _SRC_AUTOTUNE, "AUTOTUNE_ACCEL_Y_BACKOFF"),
    "autotune_target_rate_rllpit_cds":     _c(18000, _SRC_AUTOTUNE, "AUTOTUNE_TARGET_RATE_RLLPIT_CDS"),
    "autotune_target_min_rate_rllpit_cds": _c(4500, _SRC_AUTOTUNE, "AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS"),
    "autotune_target_rate_yaw_cds":        _c(9000, _SRC_AUTOTUNE, "AUTOTUNE_TARGET_RATE_YAW_CDS"),
    "autotune_target_min_rate_yaw_cds":    _c(1500, _SRC_AUTOTUNE, "AUTOTUNE_TARGET_MIN_RATE_YAW_CDS"),
    "autotune_yaw_rate_step_scale":        _c(0.75, _SRC_AUTOTUNE_RULES, "yaw rate target = 0.75 x max_rate_step"),
    "autotune_target_angle_max_rp_scale":  _c(0.5, _SRC_AUTOTUNE, "AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE of ATC_ANGLE_MAX"),
    "autotune_target_angle_min_rp_scale":  _c(1.0 / 3.0, _SRC_AUTOTUNE, "roll/pitch angle target floor, of ATC_ANGLE_MAX"),
    "autotune_target_angle_max_y_scale":   _c(1.0, _SRC_AUTOTUNE, "AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE"),
    "autotune_target_angle_min_y_scale":   _c(1.0 / 6.0, _SRC_AUTOTUNE, "yaw angle target floor, of ATC_ANGLE_MAX"),
    "autotune_angle_abort_rp_scale":       _c(2.5 / 3.0, _SRC_AUTOTUNE, "AUTOTUNE_ANGLE_ABORT_RP_SCALE"),
    "autotune_angle_neg_rp_scale":         _c(0.2, _SRC_AUTOTUNE, "AUTOTUNE_ANGLE_NEG_RP_SCALE (1/5)"),
    "autotune_success_count":    _c(4, _SRC_AUTOTUNE_H, "AUTOTUNE_SUCCESS_COUNT: consecutive passes to finish a step"),
    "autotune_level_angle_cd":   _c(250, _SRC_AUTOTUNE, "AUTOTUNE_LEVEL_ANGLE_CD"),
    "autotune_level_rate_rp_cd": _c(500, _SRC_AUTOTUNE, "AUTOTUNE_LEVEL_RATE_RP_CD"),
    "autotune_level_rate_y_cd":  _c(750, _SRC_AUTOTUNE, "AUTOTUNE_LEVEL_RATE_Y_CD"),
    "autotune_required_level_time_ms": _c(250, _SRC_AUTOTUNE, "AUTOTUNE_REQUIRED_LEVEL_TIME_MS"),
    "autotune_measure_lpf_mul":  _c(2.0, _SRC_AUTOTUNE_RULES, "twitch measurement LPF at 2 x FLTD"),
    "autotune_bounce_track_frac": _c(0.25, _SRC_AUTOTUNE_RULES, "minimum tracked once peak > 0.25 x target"),
    "autotune_early_stop_frac":  _c(0.6321, _SRC_AUTOTUNE_RULES, "step_timeout = 3 x elapsed while peak < 0.6321 target"),
    "autotune_early_stop_mul":   _c(3.0, _SRC_AUTOTUNE_RULES, "step_timeout multiplier in the early-stop rule"),
    "autotune_overshoot_aggr_scale": _c(0.5, _SRC_AUTOTUNE_RULES, "overshoot allowance = 0.5 x AGGR x target (rate P, angle P)"),
    "autotune_step_scaler_backoff": _c(0.9, _SRC_AUTOTUNE_RULES, "step_scaler *= 0.9 on a lean abort"),
    "autotune_step_scaler_min":  _c(0.2, _SRC_AUTOTUNE_RULES, "below this step_scaler: Twitch Size Determination Failed"),
    "autotune_hover_thr_min":    _c(0.1, _SRC_AUTOTUNE_RULES, "max_rate_step_bf: throttle_hover constrained low"),
    "autotune_hover_thr_max":    _c(0.5, _SRC_AUTOTUNE_RULES, "max_rate_step_bf: throttle_hover constrained high"),
    "autotune_msg_agree_pct":    _c(1.0, "docs/pid-tuning-plan.md section 2.4 (tier A)",
                                    "reconstruction vs MSG disagreement above this is a FAIL, percent"),

    # --- QuickTune (VTOL-quicktune.lua) ---------------------------------------------
    "quik_osc_smax":       _c(5.0, _SRC_QUIK, "QUIK_OSC_SMAX: PIDx.SRate above this = oscillating"),
    "quik_gain_margin":    _c(0.6, _SRC_QUIK, "QUIK_GAIN_MARGIN 60 %: gain x 0.4 when oscillating"),
    "quik_double_time_s":  _c(10.0, _SRC_QUIK, "QUIK_DOUBLE_TIME"),
    "quik_update_rate_hz": _c(40.0, _SRC_QUIK, "UPDATE_RATE_HZ"),
    "quik_rp_pi_ratio":    _c(1.0, _SRC_QUIK, "QUIK_RP_PI_RATIO: I = P / ratio when FF == 0"),
    "quik_y_pi_ratio":     _c(10.0, _SRC_QUIK, "QUIK_Y_PI_RATIO"),
    "quik_max_reduce":     _c(0.2, _SRC_QUIK, "QUIK_MAX_REDUCE 20 %"),
    "quik_yaw_p_max":      _c(0.5, _SRC_QUIK, "QUIK_YAW_P_MAX"),
    "quik_yaw_d_max":      _c(0.01, _SRC_QUIK, "QUIK_YAW_D_MAX"),
    "quik_yaw_flte_max_hz": _c(8.0, _SRC_QUIK, "YAW_FLTE_MAX"),
    "quik_fltd_mul":       _c(0.5, _SRC_QUIK, "FLTD_MUL: FLTD = 0.5 x INS_GYRO_FILTER"),
    "quik_fltt_mul":       _c(0.5, _SRC_QUIK, "FLTT_MUL: FLTT = 0.5 x INS_GYRO_FILTER"),
    "quik_default_smax":   _c(50.0, _SRC_QUIK, "DEFAULT_SMAX: any zero SMAX is set to 50"),
    "quik_angle_max_deg":  _c(10.0, _SRC_QUIK, "QUIK_ANGLE_MAX: abort on attitude error above this"),
}

def all_constants():
    """Every tuning constant table merged into one dict for `alog schema`'s
    `tune_constants`: this module's `CONSTANTS`, `tune_step.CONSTANTS`,
    `tune_ident.IDENT_CONSTANTS`, `tune_atun.ATUN_CONSTANTS` and
    `tune_fuse.FUSE_CONSTANTS`. Each table stays where it is defined; a duplicate key
    between two tables is a defect and raises. Imported lazily (those modules import
    this one)."""
    from . import tune_step, tune_ident, tune_atun, tune_fuse
    out = dict(CONSTANTS)
    for mod, attr in ((tune_step, "CONSTANTS"), (tune_ident, "IDENT_CONSTANTS"),
                      (tune_atun, "ATUN_CONSTANTS"), (tune_fuse, "FUSE_CONSTANTS")):
        for k, v in getattr(mod, attr).items():
            if k in out:
                raise KeyError(f"tuning constant {k!r} is defined twice ({mod.__name__}.{attr})")
            out[k] = v
    return out


_SRC_ACPID_DEFAULTS = ("ArduCopter libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h defaults; "
                       "reference/pid-tuning-sources.md section 2")
_SRC_ATC_DEFAULTS = "ArduCopter libraries/AC_AttitudeControl/AC_AttitudeControl.h defaults"
_SRC_INS_DEFAULTS = "ArduPilot libraries/AP_InertialSensor/AP_InertialSensor.cpp DEFAULT_GYRO_FILTER"
_SRC_MOT_DEFAULTS = "ArduPilot libraries/AP_Motors/AP_MotorsMulticopter.h AP_MOTORS_THST_HOVER_DEFAULT"
_SRC_SCHED_DEFAULTS = "ArduCopter Parameters.cpp SCHED_LOOP_RATE default"

#: Firmware defaults used when a parameter is not in the log. `GainSet.defaulted` names
#: every one that was used. cdeg/s^2 accelerations are stored in the 4.x unit and
#: converted where they are read, so the table matches the parameter documentation.
DEFAULTS = {
    "ATC_RAT_RLL_P": _c(0.135, _SRC_ACPID_DEFAULTS), "ATC_RAT_RLL_I": _c(0.135, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_RLL_D": _c(0.0036, _SRC_ACPID_DEFAULTS), "ATC_RAT_RLL_FF": _c(0.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_RLL_IMAX": _c(0.5, _SRC_ACPID_DEFAULTS), "ATC_RAT_RLL_FLTT": _c(20.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_RLL_FLTE": _c(0.0, _SRC_ACPID_DEFAULTS), "ATC_RAT_RLL_FLTD": _c(20.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_RLL_SMAX": _c(0.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_PIT_P": _c(0.135, _SRC_ACPID_DEFAULTS), "ATC_RAT_PIT_I": _c(0.135, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_PIT_D": _c(0.0036, _SRC_ACPID_DEFAULTS), "ATC_RAT_PIT_FF": _c(0.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_PIT_IMAX": _c(0.5, _SRC_ACPID_DEFAULTS), "ATC_RAT_PIT_FLTT": _c(20.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_PIT_FLTE": _c(0.0, _SRC_ACPID_DEFAULTS), "ATC_RAT_PIT_FLTD": _c(20.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_PIT_SMAX": _c(0.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_YAW_P": _c(0.18, _SRC_ACPID_DEFAULTS), "ATC_RAT_YAW_I": _c(0.018, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_YAW_D": _c(0.0, _SRC_ACPID_DEFAULTS), "ATC_RAT_YAW_FF": _c(0.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_YAW_IMAX": _c(0.5, _SRC_ACPID_DEFAULTS), "ATC_RAT_YAW_FLTT": _c(20.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_YAW_FLTE": _c(2.5, _SRC_ACPID_DEFAULTS), "ATC_RAT_YAW_FLTD": _c(20.0, _SRC_ACPID_DEFAULTS),
    "ATC_RAT_YAW_SMAX": _c(0.0, _SRC_ACPID_DEFAULTS),
    "ATC_ANG_RLL_P": _c(4.5, _SRC_ATC_DEFAULTS), "ATC_ANG_PIT_P": _c(4.5, _SRC_ATC_DEFAULTS),
    "ATC_ANG_YAW_P": _c(4.5, _SRC_ATC_DEFAULTS),
    # AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS / _Y_MAX_DEFAULT_CDSS, cdeg/s^2
    "ATC_ACCEL_R_MAX": _c(110000.0, _SRC_ATC_DEFAULTS, "cdeg/s^2"),
    "ATC_ACCEL_P_MAX": _c(110000.0, _SRC_ATC_DEFAULTS, "cdeg/s^2"),
    "ATC_ACCEL_Y_MAX": _c(27000.0, _SRC_ATC_DEFAULTS, "cdeg/s^2"),
    "ATC_RATE_FF_ENAB": _c(1.0, _SRC_ATC_DEFAULTS),
    "INS_GYRO_FILTER": _c(20.0, _SRC_INS_DEFAULTS, "Hz"),
    "MOT_THST_HOVER": _c(0.35, _SRC_MOT_DEFAULTS),
    "SCHED_LOOP_RATE": _c(400.0, _SRC_SCHED_DEFAULTS, "Hz"),
    "AUTOTUNE_AGGR": _c(0.075, _SRC_AUTOTUNE_PARAMS),
    "AUTOTUNE_MIN_D": _c(0.0005, _SRC_AUTOTUNE_PARAMS),
}

# ------------------------------------------------------------------- data model


@dataclass
class GainSet:
    """One axis' controller as configured, from PARM at the segment start."""
    axis: str
    rat_p: float
    rat_i: float
    rat_d: float
    rat_ff: float
    fltd: float
    fltt: float
    flte: float
    smax: float
    imax: float
    ang_p: float
    acc_max_dps2: float | None          # deg/s^2, converted from either spelling
    ff_enab: bool
    gyro_filter: float
    thst_hover: float | None
    loop_hz: float
    aggr: float
    gmbk: float | None                  # AUTOTUNE_GMBK when present; None on 4.x (fixed backoffs)
    min_d: float
    defaulted: list = field(default_factory=list)      # parameters that were not in the log
    param_names: dict = field(default_factory=dict)    # field -> parameter name actually read

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_log(cls, log, t, axis, aggr_override=None):
        """The gain set in force at time `t` (seconds since boot) for `axis`.

        Reads `log.param_at(name, t)` for every parameter, both spellings of the
        acceleration limit (`ATC_ACC_x_MAX` deg/s^2 preferred over `ATC_ACCEL_x_MAX`
        cdeg/s^2, converted), `SCHED_LOOP_RATE`, `INS_GYRO_FILTER`, `ATC_RATE_FF_ENAB`,
        the AutoTune parameters, and the hover throttle as the median of `CTUN.ThH` over
        the log when logged, else `MOT_THST_HOVER`. Every absent parameter is defaulted
        from `DEFAULTS` and named in `defaulted`.
        """
        if axis not in AXES:
            raise ValueError(f"axis must be one of {AXES}, not {axis!r}")
        p = _ParamAt(log, t)
        stem = RAT_STEM[axis]
        vals = {}
        for fld, suffix in (("rat_p", "P"), ("rat_i", "I"), ("rat_d", "D"), ("rat_ff", "FF"),
                            ("fltd", "FLTD"), ("fltt", "FLTT"), ("flte", "FLTE"),
                            ("smax", "SMAX"), ("imax", "IMAX")):
            vals[fld] = p.get(fld, stem + suffix)
        vals["ang_p"] = p.get("ang_p", ANG_P_PARAM[axis])

        # Acceleration limit: master `ATC_ACC_x_MAX` is deg/s^2, 4.x `ATC_ACCEL_x_MAX` is
        # cdeg/s^2. Read whichever the log has and say which; default in the 4.x unit.
        new_name, old_name = ACC_MAX_PARAMS[axis]
        v = log.param_at(new_name, t)
        if v is not None:
            vals["acc_max_dps2"] = float(v)
            p.names["acc_max_dps2"] = new_name
        else:
            v = log.param_at(old_name, t)
            if v is not None:
                vals["acc_max_dps2"] = float(v) / 100.0
                p.names["acc_max_dps2"] = old_name
            else:
                vals["acc_max_dps2"] = DEFAULTS[old_name]["value"] / 100.0
                p.names["acc_max_dps2"] = old_name
                p.defaulted.append(old_name)

        vals["ff_enab"] = bool(p.get("ff_enab", "ATC_RATE_FF_ENAB"))
        vals["gyro_filter"] = p.get("gyro_filter", "INS_GYRO_FILTER")
        vals["loop_hz"] = p.get("loop_hz", "SCHED_LOOP_RATE")

        # Hover throttle: the learned value (CTUN.ThH) is what the aircraft did; the
        # parameter is what it was told (CLAUDE.md section 2, rule 5).
        thh = log.field("CTUN", "ThH")
        if thh is not None and np.isfinite(thh).any():
            vals["thst_hover"] = float(np.nanmedian(np.asarray(thh, dtype=float)))
            p.names["thst_hover"] = "CTUN.ThH"
        else:
            vals["thst_hover"] = p.get("thst_hover", "MOT_THST_HOVER")

        vals["aggr"] = p.get("aggr", "AUTOTUNE_AGGR")
        if aggr_override is not None:
            vals["aggr"] = float(aggr_override)
            p.names["aggr"] = "override"
        gm = log.param_at("AUTOTUNE_GMBK", t)
        vals["gmbk"] = float(gm) if gm is not None else None
        p.names["gmbk"] = "AUTOTUNE_GMBK" if gm is not None else None
        vals["min_d"] = p.get("min_d", "AUTOTUNE_MIN_D")
        return cls(axis=axis, defaulted=list(p.defaulted), param_names=dict(p.names), **vals)


class _ParamAt:
    """`param_at` lookup that records every default it fell back on and the parameter
    name it read for each field."""

    def __init__(self, log, t):
        self.log, self.t = log, float(t)
        self.defaulted, self.names = [], {}

    def get(self, fld, name):
        v = self.log.param_at(name, self.t)
        self.names[fld] = name
        if v is None:
            if name not in DEFAULTS:
                raise KeyError(f"no firmware default registered for {name}; add it to tune.DEFAULTS")
            self.defaulted.append(name)
            return float(DEFAULTS[name]["value"])
        return float(v)


@dataclass
class AxisSignals:
    """One axis over one parameter-constant segment of one log. Rates in deg/s, the
    plant input `out` normalised (+-1). The PID-term arrays are None when the source
    is `RATE`."""
    axis: str
    log_name: str
    segment: tuple                      # (t0, t1) seconds since boot
    source: str                         # "PIDR" | "PIDP" | "PIDY" | "RATE"
    fs: float                           # Hz, from spectral.sample_rate
    jitter: float                       # p95 interval / median - 1
    t: np.ndarray
    tar: np.ndarray
    act: np.ndarray
    out: np.ndarray | None
    p: np.ndarray | None
    i: np.ndarray | None
    d: np.ndarray | None
    ff: np.ndarray | None
    dmod: np.ndarray | None
    srate: np.ndarray | None
    flags: np.ndarray | None
    gains: GainSet
    limited_pct: float                  # percent of samples with Flags & 1 (0 when Flags is not logged)
    clock: str = ""                     # "RATE" when `t` was restamped to the loop clock (_loop_clock)
    raw_jitter: float | None = None     # the source's own jitter before restamping

    @property
    def n(self):
        return int(len(self.t))

    @property
    def duration(self):
        return float(self.segment[1] - self.segment[0])

    def summary(self):
        """The scalar facts of this segment, JSON-safe (no arrays)."""
        return dict(axis=self.axis, log_name=self.log_name, segment=list(self.segment),
                    source=self.source, fs=self.fs, jitter=self.jitter, n=self.n,
                    clock=self.clock or self.source, raw_jitter=self.raw_jitter,
                    limited_pct=self.limited_pct, gains=self.gains.to_dict())


@dataclass
class StepResponse:
    t: np.ndarray
    mean: np.ndarray
    frames: np.ndarray                  # frames x samples, 0.5 s
    n_frames: int
    n_dropped_low: int
    metrics: dict                       # latency_s, rise_s, peak, peak_t, overshoot, bounce, settle_s, ss
    consistency: float                  # 1 - IQR(peak over frames)/median(peak), clipped to [0, 1]


@dataclass
class PlantModel:
    freqs: np.ndarray
    G: np.ndarray
    coh: np.ndarray
    n_avg: int
    band: tuple                         # (f_lo, f_hi) where coh >= gate
    k: float
    tau1: float
    tau2: float
    delay: float
    fit_rms_db: float
    fit_rms_deg: float
    coh_mean_band: float
    eps_mag_at_crossover: float         # Bendat-Piersol random error of |G|


@dataclass
class Ceiling:
    """A hard upper bound on a gain, with its origin.

    `includes_margin` False: `value` is where the loop was seen to oscillate, so fusion
    clips to QuickTune's 0.4 x ceiling. True: `value` already keeps the stability margins
    (`tune_ident.margin_ceilings`), so fusion clips to the ceiling itself."""
    param: str
    value: float
    method: str
    evidence: dict
    includes_margin: bool = False


@dataclass
class Recommendation:
    axis: str
    param: str
    current: float
    value: float
    change_pct: float
    method: str                         # "autotune-log" | "virtual-autotune" | "step-rules" | "ceiling"
    confidence: float
    components: dict                    # prior, adequacy, excitation, consistency, agreement
    evidence: dict
    note: str


@dataclass
class TuneAnalysis:
    logs: list                          # identity per log: file, firmware, board, mcu, frame, boot_time, window
    identity_ok: bool
    identity_reasons: list
    per_axis: dict                      # axis -> {signals, step, plant, autotune, ceilings}
    recommendations: list
    refusals: list                      # Refusal.to_dict()
    params: list                        # tier D Results
    constants: dict
    extra: dict = field(default_factory=dict)   # WP6: prop_in, aggr, autotune_axes, calculator table, params_by_log


@dataclass
class Refusal:
    """A structured 'cannot analyse this': a stable code, what was found, and the exact
    parameter change or flight that fixes it. Returned, never raised. Logging-related
    refusals carry the evaluated `LOGGING_REQUIREMENTS` rows in `requirements`."""
    code: str
    message: str
    fix: str
    axis: str | None = None
    log_name: str | None = None
    requirements: list = field(default_factory=list)   # logging_requirements(log) rows

    def to_dict(self):
        return dict(code=self.code, message=self.message, fix=self.fix, axis=self.axis,
                    log_name=self.log_name, requirements=list(self.requirements))

    def line(self):
        where = f" [{self.axis}]" if self.axis else ""
        return f"{self.code}{where}: {self.message} Fix: {self.fix}"


# ------------------------------------------------------------------ segmentation

def _axis_param_names(axis):
    """Parameters whose in-flight change invalidates a segment on `axis`."""
    return (RAT_STEM[axis], ANG_P_PARAM[axis]) + SEGMENT_PARAMS


def _touches(name, axis):
    stem = RAT_STEM[axis]
    return name.startswith(stem) or name == ANG_P_PARAM[axis] or name in SEGMENT_PARAMS


def _quality_gaps(log, w, subjects):
    """[(gap_start, gap_end)] from the parser's LOG_GAP diagnostic when its subject is
    one of `subjects` and the gap lies inside the window."""
    out = []
    for iss in log.quality().issues:
        if iss.code != "LOG_GAP" or iss.subject not in subjects:
            continue
        at, g = iss.detail.get("at_s"), iss.detail.get("gap_s")
        if at is None or g is None:
            continue
        if w.t0 < at + g and at < w.t1:
            out.append((float(at), float(at + g)))
    return out


def _message_gaps(log, msg, w):
    """Every gap in `msg`'s own time base inside the window, by the same rule
    `Log.quality()` applies (interval > 10 x median and > 1 s). `quality()` reports
    only the single worst gap in the log; a stalled logger usually leaves several."""
    d = log.df(msg)
    if d.empty or "t" not in d.columns:
        return []
    t = d["t"].values[w.mask(d["t"].values)]
    if len(t) < 3:
        return []
    dt = np.diff(t)
    med = float(np.median(dt))
    if not med > 0:
        return []
    idx = np.flatnonzero(dt > max(10.0 * med, 1.0))
    return [(float(t[i]), float(t[i + 1])) for i in idx]


def segment(log, w, axis, min_seconds=None, messages=None):
    """Parameter-constant, gap-free pieces of the window for one axis: [(t0, t1)].

    Split at every `log.param_changes()` entry inside the window touching that axis'
    `ATC_RAT_x_*`, `ATC_ANG_x_P`, `INS_GYRO_FILTER` or `ATC_RATE_FF_ENAB`, and at every
    logging gap in the axis' PID message or `RATE` (the parser's `LOG_GAP` diagnostic,
    plus every gap in the message's own time base by the same rule). Pieces shorter than
    `min_seconds` (default `CONSTANTS["segment_min_s"]`) are dropped.

    `messages` limits the gap scan to those message names (default: the axis' PID
    message and RATE).
    """
    if min_seconds is None:
        min_seconds = float(CONSTANTS["segment_min_s"]["value"])
    if messages is None:
        messages = (PID_MSG[axis], "RATE")
    cuts = sorted({float(t) for t, name, _old, _new in log.param_changes()
                   if w.t0 < t < w.t1 and _touches(name, axis)})
    gaps = set(_quality_gaps(log, w, set(messages)))
    for m in messages:
        gaps.update(_message_gaps(log, m, w))

    pieces = [(w.t0, w.t1)]
    for c in cuts:
        nxt = []
        for a, b in pieces:
            if a < c < b:
                nxt.extend([(a, c), (c, b)])
            else:
                nxt.append((a, b))
        pieces = nxt
    for ga, gb in sorted(gaps):
        nxt = []
        for a, b in pieces:
            if gb <= a or ga >= b:
                nxt.append((a, b))
                continue
            if ga > a:
                nxt.append((a, ga))
            if gb < b:
                nxt.append((gb, b))
        pieces = nxt
    return [(a, b) for a, b in sorted(pieces) if b - a >= min_seconds]


def _segment_detail(log, w, axis):
    """(n_param_cuts, n_gaps) - why a window produced no segments."""
    cuts = [t for t, name, _o, _n in log.param_changes() if w.t0 < t < w.t1 and _touches(name, axis)]
    msgs = (PID_MSG[axis], "RATE")
    gaps = set(_quality_gaps(log, w, set(msgs)))
    for m in msgs:
        gaps.update(_message_gaps(log, m, w))
    return len(cuts), len(gaps)


# ------------------------------------------------------------ logging requirements

#: LOG_BITMASK bits a tuning log needs (sources file section 2.1).
LOG_BITMASK_FAST_ATTITUDE = 1          # bit 0 ATTITUDE_FAST
LOG_BITMASK_PID = 4096                 # bit 12 PID
LOG_BITMASK_TUNING_BITS = LOG_BITMASK_FAST_ATTITUDE | LOG_BITMASK_PID

#: The parameters a usable tuning log needs - docs/pid-tuning-plan.md section 2.5a, the
#: single source of truth. `rule` names the predicate in `_RULES` so the required-vs-
#: current judgement is code; `required` is the text a reader sees (LOG_BITMASK's is
#: replaced by the specific corrected value once the log's own value is known).
LOGGING_REQUIREMENTS = [
    dict(param="LOG_BITMASK", rule="bitmask", required="bits 0 (1) and 12 (4096) set",
         why="bit 0 ATTITUDE_FAST + bit 12 PID: loop-rate RATE/PID logging", tier="B, C"),
    dict(param="LOG_FILE_RATEMAX", rule="zero_or_ge_loop", required="0",
         why="a non-zero cap decimates the fast stream back down (0 or >= SCHED_LOOP_RATE)", tier="B, C"),
    dict(param="LOG_BLK_RATEMAX", rule="zero", required="0",
         why="same cap for the block backend (onboard-flash boards)", tier="B, C"),
    dict(param="INS_LOG_BAT_MASK", rule="zero", required="0",
         why="batch logging doubles the write rate; separate flight", tier="B, C"),
    dict(param="LOG_FILE_BUFSIZE", rule="ge_64", required=">= 64",
         why="buffer for the doubled rate, KB; larger on boards that show LOG_GAP", tier="B, C"),
    dict(param="LOG_DISARMED", rule="report", required="any",
         why="irrelevant; the window is airborne only", tier="-"),
    dict(param="SCHED_LOOP_RATE", rule="report", required="as flown (400 default)",
         why="reported; sets the fast-log rate", tier="-"),
    dict(param="AUTOTUNE_AXES", rule="report", required="any",
         why="flying AutoTune is NOT required; read only when the log happens to hold a session (tier A)", tier="A"),
    dict(param="AUTOTUNE_AGGR", rule="report", required="as configured (0.075 default)",
         why="no AutoTune flight needed: the virtual AutoTune (tier B) applies this aggressiveness to an ordinary flight; --aggr overrides", tier="A, B"),
    dict(param="AUTOTUNE_MIN_D", rule="report", required="as configured (0.0005 default)",
         why="no AutoTune flight needed: the virtual AutoTune (tier B) uses this floor for D", tier="A, B"),
    dict(param="SID_AXIS", rule="report", required="10/11/12",
         why="optional plant-identification flight: SIDD output plus RATE.xOut input (mixer injection)",
         tier="B"),
    dict(param="SID_MAGNITUDE", rule="report", required="0.15 (yaw 0.55)",
         why="optional plant-identification flight", tier="B"),
    dict(param="SID_F_START_HZ", rule="report", required="0.5",
         why="optional plant-identification flight (firmware default)", tier="B"),
    dict(param="SID_F_STOP_HZ", rule="report", required="40",
         why="the sweep must pass the rate-loop crossover (4.5-5 Hz measured on a 10-inch quad, higher on smaller props); "
             "AnalyticTune's 5 Hz stop is for the attitude loop", tier="B"),
    dict(param="SID_T_REC", rule="report", required="70",
         why="optional plant-identification flight (firmware default)", tier="B"),
    dict(param="SID_T_FADE_IN", rule="report", required="15",
         why="optional plant-identification flight (firmware default)", tier="B"),
    dict(param="SID_T_FADE_OUT", rule="report", required="2",
         why="optional plant-identification flight (firmware default)", tier="B"),
]

LOGGING_FIX_HEADER = ("Set these ArduCopter parameters and fly the tuning profile "
                      "(SKILLS.md, 'Recommend PID gains'):")


def _rule_bitmask(cur, log):
    return int(cur) & LOG_BITMASK_TUNING_BITS == LOG_BITMASK_TUNING_BITS


def _rule_zero_or_ge_loop(cur, log):
    loop = log.param("SCHED_LOOP_RATE")
    loop = float(loop) if loop is not None else float(DEFAULTS["SCHED_LOOP_RATE"]["value"])
    return cur == 0 or cur >= loop


_RULES = {
    "bitmask": _rule_bitmask,
    "zero_or_ge_loop": _rule_zero_or_ge_loop,
    "zero": lambda cur, log: cur == 0,
    "ge_64": lambda cur, log: cur >= 64,
    "report": lambda cur, log: True,
}


def _fmt_param(v):
    """A parameter value as the GCS shows it: integers without '.0'."""
    if v is None:
        return "not in log"
    v = float(v)
    if not np.isfinite(v):
        return "NaN"
    if v == int(v) and abs(v) < 1e12:
        return str(int(v))
    return f"{v:g}"


def logging_requirements(log):
    """Evaluate `LOGGING_REQUIREMENTS` against the log's parameters.

    Returns one dict per row: param, current (None when absent), required (text; for
    LOG_BITMASK the specific corrected value `current | 1 | 4096`, e.g. 180222 -> 180223),
    ok (True / False / None when the parameter is not in the log), why, tier, rule.
    """
    out = []
    for row in LOGGING_REQUIREMENTS:
        cur = log.param(row["param"])
        cur = float(cur) if cur is not None and np.isfinite(float(cur)) else None
        required = row["required"]
        if cur is None:
            ok = None
        else:
            ok = bool(_RULES[row["rule"]](cur, log))
            if row["rule"] == "bitmask":
                required = str(int(cur) | LOG_BITMASK_TUNING_BITS)
            if cur == int(cur):
                cur = int(cur)
        out.append(dict(param=row["param"], current=cur, required=required, ok=ok,
                        why=row["why"], tier=row["tier"], rule=row["rule"]))
    return out


def format_logging_fix(rows):
    """The parameter lines of the refusal block (plan section 2.6), one per row:
    `param  current -> required  (why)` for a failing row, `param  current  ok` for a
    passing one, `param  not in log -> required  (why)` for an absent required one.
    Parameter column aligned, no trailing whitespace, deterministic."""
    width = max(len(r["param"]) for r in rows) if rows else 0
    lines = []
    for r in rows:
        cur = _fmt_param(r["current"])
        if r["ok"] is True:
            line = f"  {r['param']:<{width}}  {cur}  ok"
        elif r["ok"] is None and r["rule"] == "report":
            line = f"  {r['param']:<{width}}  {cur}"
        else:
            line = f"  {r['param']:<{width}}  {cur} -> {r['required']}  ({r['why']})"
        lines.append(line.rstrip())
    return "\n".join(lines)


def logging_fix(log):
    """(fix_text, rows): the complete fix string for a logging-related refusal, with the
    log's current values filled in, and the rows it was built from."""
    rows = logging_requirements(log)
    return LOGGING_FIX_HEADER + "\n" + format_logging_fix(rows), rows


# --------------------------------------------------------------------- extraction

def _col(d, name):
    return np.asarray(d[name].values, dtype=float) if name in d.columns else None


def _loop_clock(log, t):
    """(t_restamped, None) or (None, reason) for PIDx times `t` that failed the jitter gate.

    ArduCopter stamps PIDx with the time the record was written, part-way through the
    loop, and RATE with the loop start: on brisket-t1.bin (Brisket, 4.7.0-dev)
    PIDR read 39 % jitter while RATE read 0.6 %, the same 111 597 records, each PIDR
    0.4-1.7 ms after its own RATE tick. When every PIDx record falls inside a distinct,
    consecutive RATE tick the samples are regular and only their stamps are not, so the
    tick times are the right time base. Any skipped or doubled tick is a real drop."""
    r = log.df("RATE") if log.has("RATE") else None
    if r is None or r.empty or "t" not in r.columns:
        return None, "no RATE loop clock to restamp against"
    tr = r["t"].values.astype(float)
    idx = np.searchsorted(tr, t, side="right") - 1
    if (idx < 0).any() or len(tr) < 16:
        return None, "PIDx records start before the RATE loop clock"
    period = float(np.median(np.diff(tr)))
    step = np.diff(idx)
    bad = int(np.count_nonzero(step != 1) + np.count_nonzero(t - tr[idx] >= period))
    if bad:
        return None, (f"{bad} of {len(t)} records do not fall in distinct, consecutive RATE loop "
                      "ticks (a record was dropped or doubled), so restamping to the loop clock "
                      "would hide it")
    return tr[idx], None


def extract_axes(log, w, axes=AXES, log_name=None):
    """(signals, refusals): an `AxisSignals` per axis per segment, and every reason a
    segment or axis could not be used.

    Per axis the source is `PIDx` (`Tar`, `Act`; plant input `P+I+D+FF+DFF`, DFF
    optional) or, when absent in the window, `RATE` (`xDes`, `x`, `xOut`, with the
    PID-term arrays None). Each segment from `segment()` is gated by
    `spectral.sample_rate()` (refusal `IRREGULAR_SAMPLING`) and by
    `T["tune_pid_rate_hz"]["fail"]` (refusal `PID_RATE_TOO_LOW`). No `PIDx` and no
    `RATE` is `NO_PID_MESSAGES`; every piece dropped by a parameter change is
    `GAINS_CHANGED_IN_FLIGHT`, by gaps `IRREGULAR_SAMPLING`, by a window shorter than the
    minimum `WINDOW_TOO_SHORT`. `OUTPUT_SATURATED` (Flags & 1 above
    `T["tune_limited_pct"]["fail"]`) is advisory: the signals are still returned.

    Every logging-related refusal (`NO_PID_MESSAGES`, `PID_RATE_TOO_LOW`,
    `IRREGULAR_SAMPLING`) carries the `LOGGING_REQUIREMENTS` block with the log's own
    values filled in as its `fix`, and the evaluated rows in `requirements`.

    Refusals are returned, never raised.
    """
    if log_name is None:
        path = getattr(log, "path", None)
        log_name = os.path.basename(str(path)) if path else "log"
    signals, refusals = [], []
    rate_fail = float(T["tune_pid_rate_hz"]["fail"])
    rate_warn = float(T["tune_pid_rate_hz"]["warn"])
    limited_fail = float(T["tune_limited_pct"]["fail"])
    fix_text, req_rows = logging_fix(log)

    for axis in axes:
        if axis not in AXES:
            raise ValueError(f"axis must be one of {AXES}, not {axis!r}")
        msg = PID_MSG[axis]
        d = w.clip(log.df(msg))
        source = msg
        if d is None or d.empty or not {"Tar", "Act"} <= set(d.columns):
            dcol, acol, ocol = RATE_COLS[axis]
            d = w.clip(log.df("RATE"))
            source = "RATE"
            if d is None or d.empty or not {dcol, acol} <= set(d.columns):
                have = [m for m in (msg, "RATE") if log.has(m)]
                refusals.append(Refusal(
                    "NO_PID_MESSAGES",
                    f"no {msg} and no RATE records in the window {w.t0:.1f}-{w.t1:.1f} s"
                    + (f" ({', '.join(have)} exist outside it)" if have else
                       f" ({msg} and RATE are absent from the log)")
                    + "; LOG_BITMASK bit 0 (ATTITUDE_FAST) and bit 12 (PID) log them",
                    fix_text, axis=axis, log_name=log_name, requirements=req_rows))
                continue

        segs = segment(log, w, axis)
        if not segs:
            n_cuts, n_gaps = _segment_detail(log, w, axis)
            min_s = CONSTANTS["segment_min_s"]["value"]
            if n_cuts:
                refusals.append(Refusal(
                    "GAINS_CHANGED_IN_FLIGHT",
                    f"{n_cuts} in-flight change(s) to the {axis} rate-loop parameters leave no "
                    f"parameter-constant piece of the window {w.t0:.1f}-{w.t1:.1f} s longer than "
                    f"{min_s:.0f} s",
                    "one gain set per flight: change parameters on the ground, then fly the tuning profile",
                    axis=axis, log_name=log_name))
            elif n_gaps:
                refusals.append(Refusal(
                    "IRREGULAR_SAMPLING",
                    f"{n_gaps} logging gap(s) in {source} fragment the window {w.t0:.1f}-{w.t1:.1f} s "
                    f"into pieces shorter than {min_s:.0f} s; the logger stalled (SD card, "
                    "LOG_FILE_BUFSIZE, or batch logging on the same flight)",
                    fix_text, axis=axis, log_name=log_name, requirements=req_rows))
            else:
                refusals.append(Refusal(
                    "WINDOW_TOO_SHORT",
                    f"the window {w.t0:.1f}-{w.t1:.1f} s ({w.duration:.1f} s) is shorter than the "
                    f"{min_s:.0f} s minimum segment",
                    "fly the tuning profile (docs/pid-tuning-plan.md section 7): 30 s hover plus "
                    "60 s of stick inputs per axis", axis=axis, log_name=log_name))
            continue

        tt = d["t"].values.astype(float)
        for t0, t1 in segs:
            piece = d[(tt >= t0) & (tt <= t1)]
            t = piece["t"].values.astype(float)
            seg_txt = f"{t0:.1f}-{t1:.1f} s"
            clock, raw_jitter = "", None
            try:
                fs, st = sample_rate(t)
            except SpectralError as exc:
                why = ""
                if source != "RATE" and len(t) >= 16:
                    t_loop, why = _loop_clock(log, t)
                    if t_loop is not None:
                        try:
                            fs, st = sample_rate(t_loop)
                            dt = np.diff(t)
                            raw_jitter = float(np.percentile(dt, 95) / np.median(dt) - 1.0)
                            t, clock = t_loop, "RATE"
                        except SpectralError as exc2:
                            why = f"the RATE loop clock is irregular too ({exc2})"
                if not clock:
                    refusals.append(Refusal(
                        "IRREGULAR_SAMPLING",
                        f"{source} over {seg_txt}: {exc}" + (f"; {why}" if why else "")
                        + "; the logger stalled or the stream is too sparse (SD card, "
                        "LOG_FILE_BUFSIZE, or batch logging on the same flight)",
                        fix_text, axis=axis, log_name=log_name, requirements=req_rows))
                    continue
            if fs < rate_fail:
                refusals.append(Refusal(
                    "PID_RATE_TOO_LOW",
                    f"{source} logged at {fs:.1f} Hz over {seg_txt}; {rate_fail:.0f} Hz needed "
                    f"({rate_warn:.0f} Hz recommended)",
                    fix_text, axis=axis, log_name=log_name, requirements=req_rows))
                continue

            gains = GainSet.from_log(log, t0, axis)
            if source == "RATE":
                dcol, acol, ocol = RATE_COLS[axis]
                sig = AxisSignals(
                    axis=axis, log_name=log_name, segment=(float(t0), float(t1)), source=source,
                    fs=float(fs), jitter=float(st["jitter"]), t=t,
                    tar=_col(piece, dcol), act=_col(piece, acol), out=_col(piece, ocol),
                    p=None, i=None, d=None, ff=None, dmod=None, srate=None, flags=None,
                    gains=gains, limited_pct=0.0)
            else:
                p_, i_, d_, ff_ = (_col(piece, c) for c in ("P", "I", "D", "FF"))
                dff = _col(piece, "DFF")
                out = None
                if all(x is not None for x in (p_, i_, d_, ff_)):
                    out = p_ + i_ + d_ + ff_ + (dff if dff is not None else 0.0)
                flags = piece["Flags"].values.astype(np.int64) if "Flags" in piece.columns else None
                limited = float(100.0 * np.mean(flags & 1)) if flags is not None and len(flags) else 0.0
                sig = AxisSignals(
                    axis=axis, log_name=log_name, segment=(float(t0), float(t1)), source=source,
                    fs=float(fs), jitter=float(st["jitter"]), t=t,
                    # PIDx logs the rate PID's own rad/s; every consumer works in deg/s
                    tar=np.degrees(_col(piece, "Tar")), act=np.degrees(_col(piece, "Act")), out=out,
                    p=p_, i=i_, d=d_, ff=ff_, dmod=_col(piece, "Dmod"), srate=_col(piece, "SRate"),
                    flags=flags, gains=gains, limited_pct=limited,
                    clock=clock, raw_jitter=raw_jitter)
                if limited > limited_fail:
                    refusals.append(Refusal(
                        "OUTPUT_SATURATED",
                        f"{source} Flags bit 0 (output limited) is set on {limited:.1f} % of samples "
                        f"over {seg_txt} (fail above {limited_fail:.0f} %); the loop is nonlinear there",
                        "hover throttle and motor headroom first (`alog motors`, `alog power`); "
                        "a saturated loop cannot be tuned from its response",
                        axis=axis, log_name=log_name))
            signals.append(sig)
    return signals, refusals
