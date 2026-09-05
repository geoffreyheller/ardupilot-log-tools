"""Fail-loud parser tests on synthetic logs whose every byte is known.

Each test builds a log with tests/synthlog.py, breaks it in one specific way, and asserts
that the parser (a) still decodes everything decodable and (b) reports the breakage with
the expected integrity code and severity. A parser change that makes any of these quiet
is a regression, however well it handles healthy logs.

    python tests/test_parser_integrity.py
"""

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pytest                       # noqa: F401
except ImportError:
    import _shim as pytest

from dflog import Log, LogIntegrityError, gps_to_unix           # noqa: E402
from dflog.parser import FORMAT_CHARS, CACHE_VERSION           # noqa: E402
from synthlog import LogWriter, standard_log                    # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-test-")


def _log(blob, name="t.bin", **kw):
    path = os.path.join(TMP, name)
    with open(path, "wb") as fh:
        fh.write(blob)
    return Log(path, use_cache=False, **kw)


def _codes(log):
    return {i.code for i in log.diagnostics.issues}


# ------------------------------------------------------------------ healthy log

def test_healthy_log_has_no_issues_and_decodes_everything():
    log = _log(standard_log().bytes())
    assert log.diagnostics.ok
    assert not log.diagnostics.issues, [i.line() for i in log.diagnostics.issues]
    assert log.n_messages == 167
    assert log.types()["ATT"] == 50 and log.types()["BAT"] == 100
    assert log.bytes_parsed == log.file_size
    assert log.quality().ok


def test_format_char_scaling_is_applied_and_mult_is_not():
    log = _log(standard_log(n=3).bytes())
    att = log.df("ATT")
    assert att["Roll"].iloc[2] == pytest.approx(1.0)          # 'c' int16*100 -> /100
    assert att["Yaw"].iloc[2] == pytest.approx(6.0)           # 'C'
    assert log.units("ATT") == {"TimeUS": "s", "Roll": "deg", "Pitch": "deg", "Yaw": "deg"}
    assert log.multipliers("ATT")["TimeUS"] == pytest.approx(1e-6)
    assert att["TimeUS"].iloc[0] == 1_000_000                 # MULT 1e-6 NOT applied


def test_instances_split_and_field_aliases():
    log = _log(standard_log(n=5).bytes())
    inst = log.instances("BAT")
    assert sorted(inst) == [0, 1]
    assert len(inst[0]) == 5 and inst[1]["Volt"].iloc[0] == pytest.approx(8.0)
    assert log.field("ATT", "NoSuch", "Roll") is not None
    assert log.field("ATT", "NoSuch") is None


def test_every_format_char_round_trips():
    # FMT allows at most 16 format chars and 64 bytes of labels, so use two messages.
    w = LogWriter()
    w.fmt(200, "ALLA", "abBhHiIfdgn", "a,b,c,d,e,f,g,h,i,j,k")
    w.fmt(201, "ALLB", "NZcCeELMqQ", "l,m,n,o,p,q,r,s,t,u")
    w.msg("ALLA", a=list(range(32)), b=-5, c=250, d=-30000, e=60000, f=-2_000_000, g=4_000_000_000,
          h=1.5, i=2.25, j=0.5, k="abcd")
    w.msg("ALLB", l="sixteen chars ok", m="z" * 64, n=-12.34, o=99.99, p=-123456.78, q=1234567.89,
          r=-35.1332423, s=7, t=-(2 ** 40), u=2 ** 60)
    log = _log(w.bytes())
    assert log.diagnostics.has("NO_PARM") and len(log.diagnostics.issues) == 1, \
        [i.line() for i in log.diagnostics.issues]
    r = dict(log.raw("ALLA")[0])
    r.update(log.raw("ALLB")[0])
    assert list(r["a"]) == list(range(32))
    assert r["b"] == -5 and r["c"] == 250 and r["g"] == 4_000_000_000
    assert r["h"] == pytest.approx(1.5) and r["j"] == pytest.approx(0.5)
    assert r["k"] == "abcd" and r["l"] == "sixteen chars ok" and r["m"] == "z" * 64
    assert r["n"] == pytest.approx(-12.34) and r["o"] == pytest.approx(99.99)
    assert r["r"] == pytest.approx(-35.1332423, abs=1e-7)
    assert r["t"] == -(2 ** 40) and r["u"] == 2 ** 60
    assert set(FORMAT_CHARS) == set("abBhHiIfdgn" + "NZcCeELMqQ")


# ------------------------------------------------------------ malformed inputs

def test_empty_file():
    log = _log(b"")
    assert not log.diagnostics.ok and log.diagnostics.has("EMPTY_FILE")
    assert log.n_messages == 0


def test_not_a_dataflash_log():
    log = _log(b"This is not a log file at all, just some prose.\n" * 20)
    assert log.diagnostics.has("NO_FMT") and not log.diagnostics.ok


def test_truncated_final_message_is_a_warning_not_an_error():
    b = standard_log().bytes()
    log = _log(b[:-7])
    assert log.diagnostics.ok                        # truncation is a warning
    tr = [i for i in log.diagnostics.issues if i.code == "TRUNCATED_TAIL"]
    assert len(tr) == 1 and tr[0].severity == "warning"
    assert tr[0].detail["msg_type"] == "BAT" and tr[0].detail["bytes"] == 13
    assert log.n_messages == 166                     # every complete message kept


def test_trailing_ff_padding_is_info():
    log = _log(standard_log().bytes() + b"\xff" * 300)
    assert log.diagnostics.ok
    pad = [i for i in log.diagnostics.issues if i.code == "TRAILING_PADDING"]
    assert pad and pad[0].severity == "info" and pad[0].detail["bytes"] == 300


def test_trailing_zero_padding_is_info():
    log = _log(standard_log().bytes() + b"\x00" * 100)
    assert log.diagnostics.has("TRAILING_PADDING") and log.diagnostics.ok


def test_trailing_garbage_is_a_warning():
    log = _log(standard_log().bytes() + b"garbage at the end")
    assert log.diagnostics.has("TRAILING_GARBAGE")
    assert log.diagnostics.ok


# standard_log() ends with repeating ATT(17 B) + BAT(20 B) + BAT(20 B) = 57 bytes per step,
# so this offset is exactly on a packet boundary.
_BOUNDARY_FROM_END = 57 * 4


def test_mid_file_garbage_is_an_error_and_counted():
    b = standard_log().bytes()
    cut = len(b) - _BOUNDARY_FROM_END
    log = _log(b[:cut] + b"\x01\x02\x03\x04\x05\x06" + b[cut:])
    assert not log.diagnostics.ok
    rs = [i for i in log.diagnostics.issues if i.code == "RESYNC"]
    assert rs and rs[0].severity == "error" and rs[0].detail["bytes"] == 6
    assert log.resync_bytes == 6 and log.resync_events == 1
    assert log.n_messages == 167                     # nothing decodable was lost


def test_leading_garbage_is_a_warning_not_a_resync():
    log = _log(b"junk" + standard_log().bytes())
    assert log.diagnostics.has("LEADING_GARBAGE") and not log.diagnostics.has("RESYNC")
    assert log.resync_bytes == 0 and log.n_messages == 167


def test_unknown_message_type_is_reported_by_type_id():
    b = standard_log().bytes()
    cut = len(b) - _BOUNDARY_FROM_END
    bogus = b"\xa3\x95\xfa" + b"\x11" * 9
    log = _log(b[:cut] + bogus + b[cut:])
    unk = [i for i in log.diagnostics.issues if i.code == "UNKNOWN_MSG_TYPE"]
    assert unk and unk[0].subject == 0xfa and unk[0].severity == "error"
    assert log.diagnostics.has("RESYNC")
    assert log.n_messages == 167


def test_fmt_with_unknown_format_char():
    w = LogWriter()
    w.fmt(200, "ATT", "QccX", "TimeUS,Roll,Pitch,Yaw")
    blob = w.bytes() + b"\xa3\x95\xc8" + b"\x00" * 14
    log = _log(blob)
    assert log.diagnostics.has("FMT_UNKNOWN_FORMAT_CHAR")
    assert log.diagnostics.has("UNPARSEABLE_TYPE")
    assert not log.diagnostics.ok
    assert "ATT" not in log.messages


def test_fmt_column_count_mismatch_is_padded_and_reported():
    w = LogWriter()
    w.fmt(200, "ATT", "Qcc", "TimeUS,Roll,Pitch,Yaw")
    w.msg("ATT", TimeUS=5, Roll=1, Pitch=2)
    log = _log(w.bytes())
    assert log.diagnostics.has("FMT_COLUMN_COUNT_MISMATCH") and not log.diagnostics.ok
    assert log.columns("ATT") == ["TimeUS", "Roll", "Pitch"]
    w = LogWriter()
    w.fmt(200, "ATT", "Qccc", "TimeUS,Roll,Pitch")
    w.msg("ATT", TimeUS=5, Roll=1, Pitch=2)
    log = _log(w.bytes())
    assert log.columns("ATT") == ["TimeUS", "Roll", "Pitch", "F3"]


def test_fmt_length_mismatch_decodes_by_format_and_reports():
    w = LogWriter()
    w.fmt(200, "ATT", "Qcc", "TimeUS,Roll,Pitch", length=99)
    w.msg("ATT", TimeUS=5, Roll=1, Pitch=2)
    w.msg("ATT", TimeUS=6, Roll=1, Pitch=2)
    log = _log(w.bytes())
    lm = [i for i in log.diagnostics.issues if i.code == "FMT_LENGTH_MISMATCH"]
    assert lm and lm[0].detail == dict(declared=99, computed=15)
    assert len(log.raw("ATT")) == 2 and not log.diagnostics.has("RESYNC")


def test_fmt_redefinition_and_duplicate():
    w = LogWriter()
    w.fmt(200, "ATT", "Qcc", "TimeUS,Roll,Pitch")
    w.fmt(200, "ATT", "Qcc", "TimeUS,Roll,Pitch")            # identical re-send
    w.fmt(200, "XYZ", "QB", "TimeUS,A")                       # different definition
    w.msg("XYZ", TimeUS=1, A=2)
    log = _log(w.bytes())
    assert log.diagnostics.has("FMT_DUPLICATE")
    red = [i for i in log.diagnostics.issues if i.code == "FMT_REDEFINED"]
    assert red and red[0].severity == "warning"
    assert len(log.raw("XYZ")) == 1


def test_fmtu_for_unknown_type_and_unknown_unit_ids():
    w = LogWriter()
    w.fmt(116, "FMTU", "QBNN", "TimeUS,FmtType,UnitIds,MultIds")
    w.fmt(200, "ATT", "Qc", "TimeUS,Roll")
    w.msg("FMTU", TimeUS=1, FmtType=250, UnitIds="s-", MultIds="F-")
    w.msg("FMTU", TimeUS=2, FmtType=200, UnitIds="s~", MultIds="F-")
    w.msg("ATT", TimeUS=3, Roll=1)
    log = _log(w.bytes())
    assert log.diagnostics.has("FMTU_UNKNOWN_TYPE")
    assert log.diagnostics.has("UNIT_ID_UNKNOWN")
    assert log.diagnostics.has("MULT_ID_UNKNOWN")


def test_time_running_backwards_is_reported_per_message():
    w = LogWriter()
    w.fmt(200, "ATT", "Qc", "TimeUS,Roll")
    for t in (100, 200, 150, 300):
        w.msg("ATT", TimeUS=t, Roll=1)
    log = _log(w.bytes())
    tn = [i for i in log.diagnostics.issues if i.code == "TIME_NON_MONOTONIC"]
    assert tn and tn[0].subject == "ATT" and tn[0].severity == "warning"
    assert tn[0].detail["backwards_steps"] == 1 and tn[0].detail["largest_step_us"] == -50


def test_metadata_time_backwards_is_only_info():
    w = LogWriter()
    w.fmt(96, "PARM", "QNff", "TimeUS,Name,Value,Default")
    w.msg("PARM", TimeUS=200, Name="A", Value=1, Default=1)
    w.msg("PARM", TimeUS=100, Name="B", Value=1, Default=1)
    log = _log(w.bytes())
    tn = [i for i in log.diagnostics.issues if i.code == "TIME_NON_MONOTONIC"]
    assert tn and tn[0].severity == "info"


def test_nan_values_are_data_quality_warnings_unless_allowlisted():
    w = LogWriter()
    w.fmt(200, "BAT", "Qff", "TimeUS,Volt,Res")
    for i in range(30):
        w.msg("BAT", TimeUS=1000 * i, Volt=float("nan") if i == 3 else 16.0, Res=float("nan"))
    log = _log(w.bytes())
    q = log.quality()
    by = {i.subject: i for i in q.issues if i.code == "NAN_VALUES"}
    assert by["BAT.Volt"].severity == "warning" and by["BAT.Volt"].detail["count_values"] == 1
    assert by["BAT.Res"].severity == "info"                # allow-listed: NaN by design


def test_logging_gap_in_a_steady_stream_is_reported():
    w = LogWriter()
    w.fmt(200, "ATT", "Qc", "TimeUS,Roll")
    t = 0
    for i in range(200):
        t += 20_000 if i != 100 else 5_000_000
        w.msg("ATT", TimeUS=t, Roll=1)
    log = _log(w.bytes())
    gap = [i for i in log.quality().issues if i.code == "LOG_GAP"]
    assert gap and gap[0].detail["gap_s"] == pytest.approx(5.0, abs=0.01)


def test_duplicate_data_block_is_an_error():
    w = LogWriter()
    w.fmt(200, "ATT", "Qcc", "TimeUS,Roll,Pitch")
    vals = list(np.round(np.sin(np.arange(200) / 7.0) * 30, 2))
    vals[150:170] = vals[40:60]                         # a replayed block
    for i, v in enumerate(vals):
        w.msg("ATT", TimeUS=1000 * i, Roll=0, Pitch=v)
    log = _log(w.bytes())
    assert log.quality().has("DUPLICATE_DATA")


def test_strict_mode_raises_on_errors_only():
    b = standard_log().bytes()
    _log(b[:-7], strict=True)                             # warning only: fine
    cut = len(b) - 200
    try:
        _log(b[:cut] + b"\x01\x02\x03" + b[cut:], name="strict.bin", strict=True)
    except LogIntegrityError as exc:
        assert exc.diagnostics.has("RESYNC")
    else:
        raise AssertionError("strict mode accepted a log with a mid-file resync")


def test_no_parm_is_a_warning():
    w = LogWriter()
    w.fmt(200, "ATT", "Qc", "TimeUS,Roll")
    w.msg("ATT", TimeUS=1, Roll=1)
    log = _log(w.bytes())
    np_ = [i for i in log.diagnostics.issues if i.code == "NO_PARM"]
    assert np_ and np_[0].severity == "warning"


# ------------------------------------------------------------------ features

def test_msg_chunks_are_reassembled():
    w = LogWriter()
    w.fmt(97, "MSG", "QBBZ", "TimeUS,Id,Seq,Message")
    w.msg("MSG", TimeUS=1, Id=7, Seq=0, Message="A" * 64)
    w.msg("MSG", TimeUS=2, Id=7, Seq=1, Message="B" * 10)
    w.msg("MSG", TimeUS=3, Id=8, Seq=0, Message="short")
    log = _log(w.bytes())
    texts = [m for _, m in log.messages_text()]
    assert texts == ["A" * 64 + "B" * 10, "short"]


def test_file_records_reassemble_binary_content():
    w = LogWriter()
    w.fmt(180, "FILE", "NIBZ", "FileName,Offset,Length,Data")
    payload = bytes(range(256)) * 2                       # binary, includes NULs
    for off in range(0, len(payload), 64):
        chunk = payload[off:off + 64]
        w.msg("FILE", FileName="@SYS/x.bin", Offset=off, Length=len(chunk), Data=chunk)
    w.msg("FILE", FileName="@SYS/x.bin", Offset=64, Length=64, Data=payload[64:128])   # retried duplicate
    log = _log(w.bytes())
    assert log.files()["@SYS/x.bin"] == payload
    assert not log.diagnostics.has("STRING_NON_ASCII")


def test_instance_field_prefers_fmtu_marker():
    w = LogWriter()
    w.fmt(116, "FMTU", "QBNN", "TimeUS,FmtType,UnitIds,MultIds")
    w.fmt(117, "UNIT", "QbZ", "TimeUS,Id,Label")
    w.fmt(200, "XYZ", "QBB", "TimeUS,C,Slot")
    w.msg("UNIT", TimeUS=1, Id=ord("#"), Label="instance")
    w.msg("FMTU", TimeUS=2, FmtType=200, UnitIds="s-#", MultIds="F--")
    for i in range(6):
        w.msg("XYZ", TimeUS=10 + i, C=5, Slot=i % 2)
    log = _log(w.bytes())
    assert log.instance_field("XYZ") == "Slot"           # '#' wins over the 'C' label heuristic
    assert sorted(log.instances("XYZ")) == [0, 1]


def test_param_history_and_defaults():
    w = LogWriter()
    w.fmt(96, "PARM", "QNff", "TimeUS,Name,Value,Default")
    w.msg("PARM", TimeUS=1_000_000, Name="ATC_RAT_RLL_P", Value=0.1, Default=0.135)
    w.msg("PARM", TimeUS=50_000_000, Name="ATC_RAT_RLL_P", Value=0.12, Default=float("nan"))
    log = _log(w.bytes())
    assert log.params()["ATC_RAT_RLL_P"] == pytest.approx(0.12)
    assert log.param_defaults()["ATC_RAT_RLL_P"] == pytest.approx(0.135)
    assert log.param_at("ATC_RAT_RLL_P", 20.0) == pytest.approx(0.1)
    assert log.param_at("ATC_RAT_RLL_P", 60.0) == pytest.approx(0.12)
    ch = log.param_changes()
    assert len(ch) == 1 and ch[0][1] == "ATC_RAT_RLL_P"


def test_gps_time_conversion():
    # 2026-09-04T19:17:29Z is GPS week 2434 ... check against the known constant instead:
    # week 0, 0 ms is 1980-01-06T00:00:00Z minus 18 leap seconds.
    assert gps_to_unix(0, 0) == pytest.approx(315964800.0 - 18)
    assert gps_to_unix(1, 1000) == pytest.approx(315964800.0 + 604800 + 1 - 18)


def test_cache_is_not_written_for_a_failed_parse_and_is_rebuilt_when_stale():
    path = os.path.join(TMP, "cache.bin")
    with open(path, "wb") as fh:
        fh.write(b"not a log")
    Log(path)
    assert not os.path.exists(path + ".dfcache")
    with open(path, "wb") as fh:
        fh.write(standard_log().bytes())
    Log(path)
    assert os.path.exists(path + ".dfcache")
    with open(path + ".dfcache", "wb") as fh:
        fh.write(b"corrupt cache")
    log = Log(path)
    assert log.diagnostics.has("CACHE_REBUILT") and log.n_messages == 167
    assert CACHE_VERSION >= 4


def test_text_log_export_is_read_and_flagged():
    text = (
        "FMT, 128, 89, FMT, BBnNZ, Type,Length,Name,Format,Columns\n"
        "FMT, 200, 15, ATT, Qcc, TimeUS,Roll,Pitch\n"
        "FMT, 97, 75, MSG, QZ, TimeUS,Message\n"
        "ATT, 1000, 1.5, -2.25\n"
        "ATT, 2000, 1.6, -2.35\n"
        "MSG, 3000, ArduCopter V4.7.1 (deadbeef)\n"
        "MSG, 3100, PreArm: Check mag field (xy diff:102>100), and more\n"
        "ATT, 4000, oops, 1.0\n"
        "BOGUS, 1, 2, 3\n"
    ).encode()
    log = _log(text, name="t.log")
    assert log.source_format == "text"
    assert log.diagnostics.has("TEXT_LOG")
    assert log.diagnostics.has("TEXT_UNKNOWN_TYPE") and not log.diagnostics.ok
    assert log.diagnostics.has("TEXT_BAD_VALUE")
    att = log.df("ATT")
    assert len(att) == 3 and att["Roll"].iloc[0] == pytest.approx(1.5)   # no re-scaling
    assert log.messages_text()[1][1].startswith("PreArm: Check mag field (xy diff:102>100), and more")
    assert log.firmware().startswith("ArduCopter V4.7.1")


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
