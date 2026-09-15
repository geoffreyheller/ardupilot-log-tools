"""Check-level tests on synthetic logs: the September-2026 issues (#2, #5-#10).

Each test builds exactly the log its check needs with tests/synthlog.py, so it runs
anywhere with no flight data, then asserts on the Section the check returns. The real-log
counterparts are in tests/test_largeprop.py.

    python tests/test_checks.py
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

from dflog import Log, airborne_window                                   # noqa: E402
from dflog.analysis import check_power                                   # noqa: E402
from synthlog import LogWriter, standard_log                             # noqa: E402

TMP = tempfile.mkdtemp(prefix="dflog-checks-")


def _load(w, name):
    return Log(w.write(os.path.join(TMP, name)), use_cache=False)


def _results(sec):
    return {r.name: r for r in sec.results}


def _table(sec, name):
    t = next((t for t in sec.tables if t["name"] == name), None)
    assert t is not None, f"no table {name!r}; have {[t['name'] for t in sec.tables]}"
    return t


def _notes(sec):
    return "\n".join(sec.notes)


# ------------------------------------------------------------ issue #2

def test_charger_recalibration_formula_is_charger_over_logged():
    """BATT_AMP_PERVLT scales the reading, so logged mAh > charger mAh means PERVLT must
    come DOWN: new = old * (charger / logged). The note used to print the inverse."""
    log = _load(standard_log(n=200), "power.bin")
    sec = check_power(log, airborne_window(log, method="none"))
    text = _notes(sec)
    assert "BATT_AMP_PERVLT * (charger / logged)" in text, text
    assert "(logged / charger)" not in text


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
