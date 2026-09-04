"""dflog - a small, dependency-light ArduPilot dataflash log toolkit.

    from dflog import Log, airborne_window, mix_for, trim_decomposition

See ../CLAUDE.md for how to use this in an analysis session, and
../reference/ for the message, threshold and pitfall references.
"""
from .parser import Log, FORMAT_CHARS
from .flight import (Window, airborne_window, arm_window, mode_timeline, events,
                     esc_fundamental, EVENTS, MODES)
from .frames import MotorMix, mix_for, trim_decomposition, FRAME_CLASSES, FRAME_TYPES
from . import stats, report

__all__ = ["Log", "FORMAT_CHARS", "Window", "airborne_window", "arm_window",
           "mode_timeline", "events", "esc_fundamental", "EVENTS", "MODES",
           "MotorMix", "mix_for", "trim_decomposition", "FRAME_CLASSES",
           "FRAME_TYPES", "stats", "report"]
__version__ = "1.0.0"
