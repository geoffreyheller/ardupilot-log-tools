"""CLI contract tests: exit codes, JSON shape, error reporting - on a synthetic log so
they run anywhere, with no flight data.

    python tests/test_cli.py
"""

import io
import json
import os
import sys
import tempfile
from contextlib import redirect_stdout, redirect_stderr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import pytest                       # noqa: F401
except ImportError:
    import _shim as pytest

from dflog.cli import main, CHECK_NAMES, SCHEMA_VERSION       # noqa: E402
from synthlog import standard_log, LogWriter                   # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-cli-")
GOOD = standard_log(n=200).write(os.path.join(TMP, "good.bin"))
_b = standard_log(n=200).bytes()
with open(os.path.join(TMP, "corrupt.bin"), "wb") as _fh:
    _fh.write(_b[:len(_b) - 400] + b"\x01\x02\x03" + _b[len(_b) - 400:])
CORRUPT = os.path.join(TMP, "corrupt.bin")
with open(os.path.join(TMP, "prose.txt"), "w") as _fh:
    _fh.write("hello\n" * 100)
PROSE = os.path.join(TMP, "prose.txt")


def run(*argv):
    out, err = io.StringIO(), io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        code = main(["--no-cache"] + list(argv))
    return code, out.getvalue(), err.getvalue()


def run_json(*argv):
    code, out, err = run("--json", *argv)
    assert out.strip().startswith("{"), out[:200] + err
    return code, json.loads(out)


# ---------------------------------------------------------------- envelopes

def test_all_json_has_the_contract_shape():
    code, doc = run_json("all", GOOD)
    assert doc["schema"] == SCHEMA_VERSION and doc["command"] == "all"
    assert doc["log"]["integrity"]["ok"] is True
    assert doc["window"]["method"]
    keys = [s["key"] for s in doc["sections"]]
    assert keys == CHECK_NAMES
    for s in doc["sections"]:
        for r in s["results"]:
            assert r["status"] in ("PASS", "WARN", "FAIL", "SKIP")
            assert "summary" in r and "evidence" in r
        for t in s["tables"]:
            assert set(t) == {"name", "columns", "rows"}
    assert doc["exit_code"] == code
    import re
    assert not re.search(r"[:\[,]\s*(NaN|Infinity)\b", json.dumps(doc)), "bare NaN/Infinity in JSON"


def test_skips_are_reported_not_passed():
    code, doc = run_json("all", GOOD)
    statuses = [r["status"] for s in doc["sections"] for r in s["results"]]
    assert "SKIP" in statuses                    # a synthetic log lacks most messages
    md_code, md, _ = run("all", GOOD)
    assert "Skipped (could not run - NOT a pass)" in md
    assert md_code == code


def test_markdown_report_leads_with_integrity_and_window():
    code, md, _ = run("all", GOOD)
    assert md.startswith("# Log analysis - good.bin")
    assert "**Log integrity:**" in md
    assert "Window:" in md and "Quote this method" in md
    assert "## Verdict" in md


def test_integrity_errors_fail_the_report_and_show_at_the_top():
    code, md, _ = run("all", CORRUPT)
    assert code == 2
    assert "RESYNC" in md.split("Window:")[0]        # before the analysis, not buried
    code, doc = run_json("all", CORRUPT)
    assert doc["log"]["integrity"]["ok"] is False
    assert any(i["code"] == "RESYNC" for i in doc["log"]["integrity"]["issues"])


def test_strict_rejects_a_corrupt_log_with_exit_3():
    code, out, err = run("--strict", "all", CORRUPT)
    assert code == 3 and "rejected under --strict" in err and "RESYNC" in err
    code, doc = run_json("--strict", "all", CORRUPT)
    assert code == 3 and doc["exit_code"] == 3 and "RESYNC" in doc["error"]


def test_strict_accepts_a_log_with_only_warnings():
    b = standard_log(n=50).bytes()
    p = os.path.join(TMP, "trunc.bin")
    with open(p, "wb") as fh:
        fh.write(b[:-5])
    code, out, err = run("--strict", "info", p)
    assert code in (0, 1, 2) and "TRUNCATED_TAIL" in out


# ---------------------------------------------------------------- input errors

def test_missing_file_is_exit_3_in_both_formats():
    code, out, err = run("all", os.path.join(TMP, "nope.bin"))
    assert code == 3 and "no such file" in err and out == ""
    code, doc = run_json("all", os.path.join(TMP, "nope.bin"))
    assert code == 3 and doc["exit_code"] == 3 and "no such file" in doc["error"]


def test_non_log_file_is_exit_3_with_diagnostics():
    code, out, err = run("all", PROSE)
    assert code == 3 and "NO_FMT" in err


def test_unknown_message_in_dump_lists_what_exists():
    code, out, err = run("dump", GOOD, "NOPE")
    assert code == 3 and "ATT" in err and "BAT" in err
    code, out, err = run("dump", GOOD, "ATT", "--fields", "t,Nope")
    assert code == 3 and "Nope" in err


def test_unknown_check_in_compare_is_exit_3():
    code, out, err = run("compare", GOOD, GOOD, "--checks", "motors,bogus")
    assert code == 3 and "bogus" in err


def test_bad_explicit_window_is_exit_3():
    code, out, err = run("vibe", GOOD, "--window", "50:10")
    assert code == 3 and "end must be after start" in err


# ---------------------------------------------------------------- commands

def test_info_and_integrity_and_types_and_fields():
    code, doc = run_json("info", GOOD)
    assert doc["info"]["firmware"].startswith("ArduCopter") and doc["info"]["n_params"] == 2
    assert doc["coverage"]["key"] == "coverage"
    code, doc = run_json("integrity", GOOD)
    assert code == 0 and doc["structure"]["ok"] and doc["quality"]["ok"]
    code, doc = run_json("types", GOOD)
    names = {p["name"] for p in doc["present"]}
    assert {"ATT", "BAT", "PARM", "MSG"} <= names
    code, doc = run_json("fields", GOOD, "ATT")
    assert [f["name"] for f in doc["fields"]] == ["TimeUS", "Roll", "Pitch", "Yaw"]
    assert doc["fields"][1]["unit"] == "deg"
    code, out, err = run("fields", GOOD, "NOPE")
    assert code == 3


def test_dump_csv_and_json_with_window_and_decimation():
    code, out, err = run("dump", GOOD, "ATT", "--fields", "t,Roll", "--window", "none", "--every", "50")
    lines = out.strip().splitlines()
    assert lines[0] == "t,Roll" and len(lines) == 5
    code, doc = run_json("dump", GOOD, "BAT", "--instance", "1", "--window", "none", "--limit", "3")
    assert doc["n"] == 3 and all(r["Inst"] == 1 for r in doc["rows"])
    code, out, err = run("dump", GOOD, "BAT", "--instance", "7")
    assert code == 3 and "no instance 7" in err


def test_params_and_diff():
    code, out, err = run("params", GOOD)
    assert "FRAME_CLASS,1" in out
    pf = os.path.join(TMP, "snap.param")
    with open(pf, "w") as fh:
        fh.write("FRAME_CLASS,2\nFRAME_TYPE,1\nEXTRA_ONLY,5\n# comment\nnot a param line\n")
    code, doc = run_json("params", GOOD, "--diff", pf)
    notes = {d["name"]: d["note"] for d in doc["differences"]}
    assert notes == {"FRAME_CLASS": "changed", "EXTRA_ONLY": "only in file"}
    assert doc["unparsed_lines"] == 1
    code, out, err = run("params", GOOD, "--diff", os.path.join(TMP, "none.param"))
    assert code == 3


def test_compare_runs_identical_code_on_both():
    code, doc = run_json("compare", GOOD, GOOD, "--checks", "vibe,motors", "--window", "none")
    assert len(doc["logs"]) == 2 and doc["checks"]
    code, out, err = run("compare", GOOD, GOOD, "--checks", "vibe", "--window", "none")
    assert "Like-for-like" in out


def test_schema_lists_checks_and_thresholds_with_sources():
    code, doc = run_json("schema")
    assert doc["checks"] == CHECK_NAMES
    assert all("source" in v for v in doc["thresholds"].values())
    assert "3" in doc["exit_codes"] or 3 in doc["exit_codes"]


def test_fft_refuses_when_no_signal_and_says_what_exists():
    code, out, err = run("fft", GOOD)
    assert code == 3 and "FFT refused" in err


def test_fft_on_a_synthetic_gyro_finds_the_injected_tone_and_warns_on_jitter():
    import numpy as np
    w = LogWriter()
    w.fmt(96, "PARM", "QNff", "TimeUS,Name,Value,Default")
    w.fmt(200, "IMU", "QBfff", "TimeUS,I,GyrX,GyrY,GyrZ")
    w.msg("PARM", TimeUS=1, Name="X", Value=1, Default=1)
    fs = 400.0
    n = 4000
    t = np.arange(n) / fs
    tone = 0.5 * np.sin(2 * np.pi * 57.0 * t)
    for i in range(n):
        w.msg("IMU", TimeUS=int(1e6 + t[i] * 1e6), I=0, GyrX=tone[i], GyrY=0.01 * np.sin(i), GyrZ=0.0)
    p = w.write(os.path.join(TMP, "tone.bin"))
    code, doc = run_json("fft", p, "--window", "none", "--axes", "GyrX")
    pk = doc["fft"]["peaks"]["GyrX"][0]
    assert abs(pk["freq_hz"] - 57.0) < 1.0
    assert doc["fft"]["fs_hz"] == pytest.approx(400.0, rel=0.01)
    assert code == 1                                    # warns: no ESC telemetry for orders
    # now with irregular timing: must refuse rather than transform
    w2 = LogWriter()
    w2.fmt(200, "IMU", "QBfff", "TimeUS,I,GyrX,GyrY,GyrZ")
    rng = np.random.default_rng(0)
    tt = np.cumsum(rng.uniform(1000, 6000, size=500)).astype(int)
    for i in range(500):
        w2.msg("IMU", TimeUS=int(tt[i]), I=0, GyrX=0.1, GyrY=0.0, GyrZ=0.0)
    p2 = w2.write(os.path.join(TMP, "jitter.bin"))
    code, out, err = run("fft", p2, "--window", "none")
    assert code == 3 and "irregular" in err


def test_files_command_extracts_embedded_files():
    w = standard_log(n=5)
    w.fmt(180, "FILE", "NIBZ", "FileName,Offset,Length,Data")
    w.msg("FILE", FileName="@SYS/a.txt", Offset=0, Length=5, Data=b"hello")
    p = w.write(os.path.join(TMP, "files.bin"))
    out_dir = os.path.join(TMP, "extracted")
    code, doc = run_json("files", p, "--out", out_dir)
    assert doc["files"] == [dict(name="@SYS/a.txt", bytes=5)]
    assert open(os.path.join(out_dir, "SYS_a.txt"), "rb").read() == b"hello"
    code, out, err = run("files", GOOD)
    assert code == 3


def test_json_flag_position_is_flexible():
    code1, out1, _ = run("--json", "types", GOOD)
    code2, out2, _ = run("types", GOOD, "--json")
    assert json.loads(out1)["command"] == json.loads(out2)["command"] == "types"


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
