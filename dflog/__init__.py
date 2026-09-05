"""dflog - an agent-first, fail-loud ArduPilot dataflash log toolkit.

    from dflog import Log, airborne_window, mix_for, trim_decomposition

    log = Log("flight.bin")            # parses once, caches beside the log
    log.diagnostics.ok                 # False if anything in the file was wrong
    w   = airborne_window(log, "rpm")  # the window, with the method that chose it

See ../CLAUDE.md for how to use this in an analysis session, ../RULES.md for the
contract every output obeys, and ../reference/ for the message, threshold, pitfall and
integrity-code references.
"""
from .parser import Log, FORMAT_CHARS, Diagnostics, Issue, LogIntegrityError, gps_to_unix
from .flight import (Window, airborne_window, arm_window, mode_timeline, events,
                     esc_fundamental, hover_chunks, EVENTS, MODES, MODE_REASONS)
from .frames import MotorMix, mix_for, trim_decomposition, motor_channels, FRAME_CLASSES, FRAME_TYPES
from . import stats, report, spectral

__all__ = ["Log", "FORMAT_CHARS", "Diagnostics", "Issue", "LogIntegrityError", "gps_to_unix",
           "Window", "airborne_window", "arm_window", "mode_timeline", "events", "esc_fundamental",
           "hover_chunks", "EVENTS", "MODES", "MODE_REASONS",
           "MotorMix", "mix_for", "trim_decomposition", "motor_channels", "FRAME_CLASSES",
           "FRAME_TYPES", "stats", "report", "spectral"]
__version__ = "2.0.0"
