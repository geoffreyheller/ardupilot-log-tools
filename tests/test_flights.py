"""Flight segmentation: a log may hold more than one flight, and the window must say so.

Issue #1: `airborne_window` had three separate ad-hoc constructions of "the airborne
part" and none of them knew a log can hold two flights. `rpm` and `throttle` returned
min..max of the airborne mask and so *spanned* the ground time between flights; `ev` and
`arm` silently analysed only the first. Both are contract violations - RULES.md §1 (fail
loudly, never repair silently) and §2 (state the number, the window and the source).

Everything here is built with tests/synthlog.py, so it runs anywhere with no flight data.

    python tests/test_flights.py
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pytest                       # noqa: F401
except ImportError:                     # PyPI is blocked in the sandbox; see tests/_shim.py
    import _shim as pytest

from dflog import Log, airborne_window, arm_window, flights, hover_chunks      # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-flights-")

RPM_FLYING = 8400.0          # 140 Hz fundamental, well above the 90 Hz floor
RPM_IDLE = 0.0
SPAN = 180.0                 # seconds of log
HZ = 10.0                    # ESC / CTUN / RCIN sample rate in these fixtures


# ------------------------------------------------------------------- fixtures

def _writer():
    """A minimal but well-formed copter log: firmware banner, frame params, and the
    four message types the four window detectors read."""
    from synthlog import LogWriter
    w = LogWriter()
    w.fmt(96, "PARM", "QNff", "TimeUS,Name,Value,Default")
    w.fmt(97, "MSG", "QZ", "TimeUS,Message")
    w.fmt(64, "EV", "QB", "TimeUS,Id")
    w.fmt(202, "ESC", "QBff", "TimeUS,Instance,RPM,Volt")
    w.fmt(203, "CTUN", "Qf", "TimeUS,ThO")
    w.fmt(204, "ARM", "QBBH", "TimeUS,ArmState,ArmChecks,Forced")
    w.fmt(205, "MODE", "QBBB", "TimeUS,ModeNum,Rsn,ThrCrs")
    w.fmt(206, "RCIN", "QHHHH", "TimeUS,C1,C2,C3,C4")
    w.msg("MSG", TimeUS=20, Message="ArduCopter V4.7.1 (deadbeef)")
    w.msg("PARM", TimeUS=30, Name="FRAME_CLASS", Value=1.0, Default=1.0)
    w.msg("PARM", TimeUS=31, Name="FRAME_TYPE", Value=1.0, Default=1.0)
    return w


def _make(spans, name, span=SPAN, ev=True, esc=True, ctun=True, arm=None,
          rpm_dips=(), extra_events=(), rpm_flying=RPM_FLYING, rpm_climbs=(), rpm_climb=None,
          ev_spans=None, rpm_idle=RPM_IDLE):
    """A log whose aircraft flew during `spans` [(t0, t1), ...] and sat on the ground
    otherwise.

    ev/esc/ctun switch each detector's evidence on or off independently, so a test can
    exercise one detector in isolation. `arm` is [(t0, t1), ...] of armed periods.
    `rpm_dips` are [(t0, t1)] inside a span where the fundamental drops below the floor.
    `rpm_flying` is the cruising RPM; `rpm_climbs` are [(t0, t1)] where it rises to
    `rpm_climb` instead (a large-prop aircraft only clears a fixed 90 Hz floor in climbs).
    `ev_spans` overrides the flights the EV stream reports, when it should disagree with
    the ESC stream. `rpm_idle` is the RPM while on the ground (armed idle, or 0).
    """
    w = _writer()
    if ev:
        for t0, t1 in (spans if ev_spans is None else ev_spans):
            w.msg("EV", TimeUS=int(t0 * 1e6), Id=28)          # NOT_LANDED
            if t1 is not None:
                w.msg("EV", TimeUS=int(t1 * 1e6), Id=18)      # LAND_COMPLETE
    for t, i in extra_events:
        w.msg("EV", TimeUS=int(t * 1e6), Id=i)
    for t0, t1 in (arm or ()):
        w.msg("EV", TimeUS=int(t0 * 1e6), Id=10)              # ARMED
        w.msg("ARM", TimeUS=int(t0 * 1e6), ArmState=1, ArmChecks=0, Forced=0)
        if t1 is not None:
            w.msg("EV", TimeUS=int(t1 * 1e6), Id=11)          # DISARMED
            w.msg("ARM", TimeUS=int(t1 * 1e6), ArmState=0, ArmChecks=0, Forced=0)

    def airborne(t):
        if any(a <= t <= (b if b is not None else span) for a, b in spans):
            return not any(a <= t <= b for a, b in rpm_dips)
        return False

    n = int(span * HZ)
    for i in range(n):
        t = i / HZ
        up = airborne(t)
        if esc:
            rpm = rpm_idle
            if up:
                rpm = rpm_climb if any(a <= t <= b for a, b in rpm_climbs) else rpm_flying
            for inst in range(4):
                w.msg("ESC", TimeUS=int(t * 1e6), Instance=inst, RPM=rpm, Volt=7.8)
        if ctun:
            w.msg("CTUN", TimeUS=int(t * 1e6), ThO=0.5 if up else 0.0)
    p = os.path.join(TMP, name)
    w.write(p)
    return Log(p, use_cache=False)


# Two flights, 10-60 s and 120-170 s, with 60 s of ground time between them. This is the
# log from the issue, with CTUN and ARM added so all four detectors are exercised rather
# than falling back.
TWO = _make([(10.0, 60.0), (120.0, 170.0)], "two_flights.bin",
            arm=[(5.0, 65.0), (115.0, 175.0)])

# One flight: the shape every pinned figure and every written report was taken on.
ONE = _make([(10.0, 60.0)], "one_flight.bin", arm=[(5.0, 65.0)])

# Nothing a detector can use.
BARE = _make([], "bare.bin", ev=False, esc=False, ctun=False, span=60.0)


def _span(w, t0, t1, tol=0.11):
    """The window covers exactly t0..t1 (tolerance: one 10 Hz sample)."""
    assert abs(w.t0 - t0) <= tol and abs(w.t1 - t1) <= tol, \
        f"expected {t0}-{t1} s, got {w.t0:.2f}-{w.t1:.2f} s via {w.method}"


# ------------------------------------------------ Group A: the bug (issue #1)

def test_rpm_window_does_not_span_the_gap_between_flights():
    """The headline bug: 10-170 s, of which 60 s (37 %) was sitting on the ground."""
    w = airborne_window(TWO, method="rpm")
    _span(w, 10.0, 60.0)
    assert w.t1 != pytest.approx(170.0, abs=0.5)


def test_throttle_window_does_not_span_the_gap():
    w = airborne_window(TWO, method="throttle")
    _span(w, 10.0, 60.0)


def test_arm_window_does_not_span_the_gap():
    w = airborne_window(TWO, method="arm")
    _span(w, 5.0, 65.0)


def test_ev_window_names_the_flight_it_chose():
    """RULES §2: the method string is the one thing users are told to quote. On a log
    with two flights it must say which one this is."""
    w = airborne_window(TWO, method="auto")
    assert "flight 1 of 2" in w.method, w.method
    w = airborne_window(TWO, method="rpm")
    assert "flight 1 of 2" in w.method, w.method


def test_flights_finds_both_segments_by_ev():
    fl = flights(TWO, method="ev")
    assert len(fl) == 2, [str(f) for f in fl]
    _span(fl[0], 10.0, 60.0)
    _span(fl[1], 120.0, 170.0)
    assert [f.index for f in fl] == [1, 2]


def test_flights_finds_both_segments_by_rpm():
    fl = flights(TWO, method="rpm")
    assert len(fl) == 2, [str(f) for f in fl]
    _span(fl[0], 10.0, 60.0)
    _span(fl[1], 120.0, 170.0)


def test_flights_finds_both_segments_by_throttle_and_arm():
    assert len(flights(TWO, method="throttle")) == 2
    fl = flights(TWO, method="arm")
    assert len(fl) == 2
    _span(fl[0], 5.0, 65.0)


def test_flight_selection():
    w = airborne_window(TWO, method="rpm", flight=2)
    _span(w, 120.0, 170.0)
    assert "flight 2 of 2" in w.method, w.method
    _span(airborne_window(TWO, method="ev", flight=1), 10.0, 60.0)


def test_flight_out_of_range_is_an_error():
    try:
        airborne_window(TWO, method="rpm", flight=3)
    except ValueError as exc:
        assert "2" in str(exc), str(exc)
    else:
        raise AssertionError("flight=3 on a two-flight log must raise")
    try:
        airborne_window(TWO, method="rpm", flight=0)
    except ValueError:
        pass
    else:
        raise AssertionError("flight=0 must raise: the index is 1-based")


def test_flight_with_explicit_window_is_an_error():
    """An explicit window and a flight index contradict each other. RULES §1: that is an
    error, not a silent reinterpretation of one of them."""
    for method in ("none", "120:180"):
        try:
            airborne_window(TWO, method=method, flight=1)
        except ValueError as exc:
            assert "flight" in str(exc).lower(), str(exc)
        else:
            raise AssertionError(f"method={method!r} with flight=1 must raise")


def test_window_carries_the_segment_list():
    """Disclosure: the window knows about the flights it did not cover, so every caller
    can report them without re-deriving them."""
    w = airborne_window(TWO, method="rpm")
    assert w.index == 1 and len(w.segments) == 2
    assert w.to_dict()["n_flights"] == 2 and w.to_dict()["flight_index"] == 1


def test_longest_flight_wins_when_they_differ():
    log = _make([(10.0, 40.0), (100.0, 170.0)], "uneven.bin")
    w = airborne_window(log, method="rpm")
    _span(w, 100.0, 170.0)
    assert "flight 2 of 2" in w.method, w.method


# --------------------------------------- Group B: guards (must pass unchanged)

def test_single_flight_window_is_unchanged():
    """A one-flight log's window and *method string* are byte-identical to before this
    change: no `flight k of n` suffix, so existing reports and docs stay valid.

    Amended for issue #3: the `rpm` floor is now derived from the log unless given, and
    the method string says what it derived. Passing `hz_floor=90.0` explicitly reproduces
    the pre-#3 string exactly, which is what a report written before then quoted."""
    w = airborne_window(ONE, method="ev")
    _span(w, 10.0, 60.0)
    assert w.method == "EV NOT_LANDED->LAND_COMPLETE"
    w = airborne_window(ONE, method="rpm", hz_floor=90.0)
    _span(w, 10.0, 60.0)
    assert w.method == "ESC fundamental > 90 Hz"
    assert w.note == "same definition regardless of land-detector state"
    w = airborne_window(ONE, method="rpm")
    _span(w, 10.0, 60.0)
    assert w.method.startswith("ESC fundamental > "), w.method
    assert "spinning median" in w.method, w.method
    w = airborne_window(ONE, method="throttle")
    assert w.method == "CTUN.ThO > 0.15"
    assert airborne_window(ONE, method="auto").method == "EV NOT_LANDED->LAND_COMPLETE"


# ------------------------------------------ Group D: the rpm floor (issue #3)
#
# A 10-inch / 380 KV quad hovers near 84 Hz (5040 RPM). A fixed 90 Hz floor sits *above*
# that, so the mask only latches in climbs and every cruise between two climbs reads as a
# landing. This synthetic aircraft cruises at 84 Hz and climbs to 100 Hz three times.

RPM_HOVER_LARGE = 5040.0      # 84 Hz
RPM_CLIMB_LARGE = 6000.0      # 100 Hz
LARGE = _make([(10.0, 170.0)], "largeprop.bin",
              rpm_flying=RPM_HOVER_LARGE, rpm_climb=RPM_CLIMB_LARGE,
              rpm_climbs=[(12.0, 30.0), (70.0, 90.0), (140.0, 150.0)])


def test_fixed_90hz_floor_splits_a_large_prop_flight():
    """The failure from the issue, kept as the explicit-floor behaviour: with the floor
    pinned at 90 Hz the aircraft is 'airborne' only in its three climbs."""
    fl = flights(LARGE, method="rpm", hz_floor=90.0)
    assert len(fl) == 3, [str(f) for f in fl]
    assert all("> 90 Hz" in f.method for f in fl)


def test_derived_floor_finds_one_flight_on_a_large_prop_log():
    """Default floor: 60 % of the spinning median, so it scales with the aircraft."""
    fl = flights(LARGE, method="rpm")
    assert len(fl) == 1, [str(f) for f in fl]
    _span(fl[0], 10.0, 170.0)
    assert len(flights(LARGE, method="ev")) == 1
    w = airborne_window(LARGE, method="rpm")
    _span(w, 10.0, 170.0)
    assert w.hz_floor == pytest.approx(0.6 * 84.0, abs=0.5), w.hz_floor
    assert "spinning median" in w.method and "50." in w.method, w.method


def test_derived_floor_still_separates_two_flights_on_a_small_prop_log():
    """The derivation must not regress the small-prop case: 140 Hz cruise, 60 s on the
    ground between two flights, and the floor lands at 84 Hz - above idle, below cruise."""
    fl = flights(TWO, method="rpm")
    assert len(fl) == 2, [str(f) for f in fl]
    _span(fl[0], 10.0, 60.0)
    _span(fl[1], 120.0, 170.0)
    assert fl[0].hz_floor == pytest.approx(84.0, abs=0.5)


def test_derived_floor_clears_an_armed_idle_on_the_ground():
    """Armed idle at MOT_SPIN_ARM spins the props (~25 Hz here) without flying. The
    derived floor must sit above that, so ground idle is not a flight."""
    log = _make([(30.0, 150.0)], "idle.bin", rpm_flying=RPM_HOVER_LARGE, rpm_idle=1500.0,
                arm=[(5.0, 175.0)])
    fl = flights(log, method="rpm")
    assert len(fl) == 1, [str(f) for f in fl]
    _span(fl[0], 30.0, 150.0)


def test_explicit_floor_is_honoured_and_named():
    fl = flights(LARGE, method="rpm", hz_floor=60.0)
    assert len(fl) == 1 and fl[0].method == "ESC fundamental > 60 Hz", fl[0].method
    assert fl[0].hz_floor == 60.0


def test_hover_window_method_picks_the_longest_hover_chunk():
    """issue #10: `--window hover` is the CLI route to hover_chunks()."""
    w = _writer()
    w.msg("EV", TimeUS=int(10.0 * 1e6), Id=28)
    w.msg("EV", TimeUS=int(170.0 * 1e6), Id=18)
    w.msg("MODE", TimeUS=int(5.0 * 1e6), ModeNum=0, Rsn=1, ThrCrs=0)        # STABILIZE
    w.msg("MODE", TimeUS=int(40.0 * 1e6), ModeNum=5, Rsn=1, ThrCrs=0)       # LOITER 40-60
    w.msg("MODE", TimeUS=int(60.0 * 1e6), ModeNum=0, Rsn=1, ThrCrs=0)
    w.msg("MODE", TimeUS=int(100.0 * 1e6), ModeNum=5, Rsn=1, ThrCrs=0)      # LOITER 100-150
    w.msg("MODE", TimeUS=int(150.0 * 1e6), ModeNum=0, Rsn=1, ThrCrs=0)
    for i in range(int(SPAN * HZ)):
        t = i / HZ
        up = 10.0 <= t <= 170.0
        for inst in range(4):
            w.msg("ESC", TimeUS=int(t * 1e6), Instance=inst, RPM=RPM_FLYING if up else 0.0, Volt=7.8)
        w.msg("RCIN", TimeUS=int(t * 1e6), C1=1500, C2=1500, C3=1500, C4=1500)
    log = Log(w.write(os.path.join(TMP, "hover_window.bin")), use_cache=False)
    win = airborne_window(log, method="hover")
    _span(win, 100.0, 150.0)
    assert win.method.startswith("hover chunk 2 of 2"), win.method
    assert "LOITER" in win.method
    # a log with no hover at all falls back, and says so
    lo, hi = LARGE.duration()
    fb = airborne_window(LARGE, method="hover")
    assert (fb.t0, fb.t1) == (lo, hi) and "FALLBACK" in fb.method, fb.method


def test_fallback_window_is_unchanged():
    lo, hi = BARE.duration()
    w = airborne_window(BARE, method="rpm")
    assert (w.t0, w.t1) == (lo, hi)
    assert w.method == "whole log (FALLBACK: method 'rpm' found no airborne signal)"


def test_arm_window_strings_unchanged():
    """arm_window() is exported and documented as the *armed* span, ground time
    included. It keeps spanning, and it keeps its strings."""
    w = arm_window(TWO)
    assert w.method == "EV ARMED->DISARMED"
    _span(w, 5.0, 175.0)
    assert arm_window(BARE).method == "whole log (no ARMED event)"


def test_explicit_and_none_windows_unchanged():
    w = airborne_window(TWO, method="120:180")
    assert (w.t0, w.t1) == (120.0, 180.0) and w.method == "explicit 120:180 s"
    lo, hi = TWO.duration()
    w = airborne_window(TWO, method="none")
    assert (w.t0, w.t1) == (lo, hi) and w.method == "whole log (requested)"
    try:
        airborne_window(TWO, method="50:10")
    except ValueError as exc:
        assert "end must be after start" in str(exc)
    else:
        raise AssertionError("a backwards explicit window must raise")


def test_pad_still_applies_and_still_raises():
    w = airborne_window(ONE, method="rpm", pad=5.0)
    _span(w, 15.0, 55.0)
    assert "padded" in w.note
    try:
        airborne_window(ONE, method="rpm", pad=30.0)
    except ValueError as exc:
        assert "removes the whole" in str(exc)
    else:
        raise AssertionError("a pad that removes the whole window must raise")


# ------------------------------------------------------- Group C: edge cases

def test_bounced_landing_is_one_flight():
    """LAND_COMPLETE then NOT_LANDED a second later is a bounce, not two flights.
    This is what gap_seconds is for."""
    log = _make([(10.0, 60.0), (61.0, 120.0)], "bounce.bin")
    fl = flights(log, method="ev")
    assert len(fl) == 1, [str(f) for f in fl]
    _span(fl[0], 10.0, 120.0)
    assert len(flights(log, method="rpm")) == 1


def test_unterminated_flight_closes_at_log_end():
    """A log that ends while still airborne: one segment, closed at the end of the log,
    and the note says so rather than pretending the aircraft landed."""
    log = _make([(10.0, None)], "unterminated.bin")
    fl = flights(log, method="ev")
    assert len(fl) == 1
    hi = log.duration()[1]
    assert abs(fl[0].t1 - hi) < 0.2
    assert "end of log" in fl[0].note.lower(), fl[0].note


def test_mid_flight_rpm_dip_is_one_flight():
    """A descent that drops the fundamental below the floor for two seconds must not
    split the flight - and must not shorten the window either."""
    log = _make([(10.0, 60.0)], "dip.bin", rpm_dips=[(30.0, 32.0)])
    fl = flights(log, method="rpm")
    assert len(fl) == 1, [str(f) for f in fl]
    _span(fl[0], 10.0, 60.0)


def test_short_blip_is_not_a_flight():
    """Two seconds of spin-up on the bench is below min_seconds and is not a flight."""
    log = _make([(10.0, 60.0), (100.0, 102.0), (120.0, 170.0)], "blip.bin", ev=False)
    fl = flights(log, method="rpm")
    assert len(fl) == 2, [str(f) for f in fl]
    _span(fl[0], 10.0, 60.0)
    _span(fl[1], 120.0, 170.0)


def test_equal_length_flights_pick_the_earliest():
    """Documented tie-break, so the choice is reproducible."""
    w = airborne_window(TWO, method="rpm")
    assert w.index == 1
    _span(w, 10.0, 60.0)


def test_hover_chunks_do_not_straddle_flights():
    """A mode that never changes and centred sticks would otherwise yield one 'hover'
    chunk covering both flights and the ground time between them."""
    w = _writer()
    spans = [(10.0, 60.0), (120.0, 170.0)]
    for t0, t1 in spans:
        w.msg("EV", TimeUS=int(t0 * 1e6), Id=28)
        w.msg("EV", TimeUS=int(t1 * 1e6), Id=18)
    w.msg("MODE", TimeUS=int(5.0 * 1e6), ModeNum=5, Rsn=1, ThrCrs=0)      # LOITER, never changes
    for i in range(int(SPAN * HZ)):
        t = i / HZ
        up = any(a <= t <= b for a, b in spans)
        for inst in range(4):
            w.msg("ESC", TimeUS=int(t * 1e6), Instance=inst,
                  RPM=RPM_FLYING if up else RPM_IDLE, Volt=7.8)
        w.msg("RCIN", TimeUS=int(t * 1e6), C1=1500, C2=1500, C3=1500, C4=1500)
    p = w.write(os.path.join(TMP, "hover.bin"))
    log = Log(p, use_cache=False)
    chunks = hover_chunks(log)
    assert chunks, "expected at least one hover chunk"
    for c in chunks:
        assert not (c.t0 < 60.5 and c.t1 > 119.5), \
            f"chunk {c.t0:.1f}-{c.t1:.1f} s straddles the gap between two flights"


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
