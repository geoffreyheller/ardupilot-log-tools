#!/usr/bin/env python3
"""Vendor pymavlink from GitHub into ./vendor/, with the MAVLink dialects generated.
Cross-platform replacement for bootstrap_pymavlink.sh (which still works where bash does).

You do NOT need this for anything in alog.py. Run it when you want:
  - mavextra.py            WMM expected earth field, magfit, stateful filters
  - tools/mavfft_isb.py    the reference batch-IMU FFT tool
  - MAVExplorer            interactive graphing (needs MAVProxy too)

Why a script: pymavlink's repo ships no generated dialect modules (dialects/.gitignore
excludes *.py) and message_definitions/ lives in a different repo (ArduPilot/mavlink).
DFReader.py imports mavutil, which calls set_dialect() at import time and dies without
them, so a plain clone is not importable.

Requires: git, python3, lxml (pip install lxml).

    python tools/bootstrap_pymavlink.py
"""
from __future__ import annotations

import glob
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENDOR = os.path.join(ROOT, "vendor")


def run(*cmd, cwd=None):
    print(">>", " ".join(cmd))
    subprocess.run(cmd, cwd=cwd, check=True)


def main():
    if shutil.which("git") is None:
        print("error: git is not on PATH", file=sys.stderr)
        return 1
    os.makedirs(VENDOR, exist_ok=True)
    pym = os.path.join(VENDOR, "pymavlink")
    mav = os.path.join(VENDOR, "mavlink")
    if not os.path.isdir(pym):
        run("git", "clone", "--depth", "1", "-q", "https://github.com/ArduPilot/pymavlink.git", pym)
    if not os.path.isdir(mav):
        run("git", "clone", "--depth", "1", "--filter=blob:none", "--sparse", "-q",
            "https://github.com/ArduPilot/mavlink.git", mav)
        run("git", "-C", mav, "sparse-checkout", "set", "message_definitions")
    # mavgen resolves <include> relative to the target directory, so copy ALL the XMLs.
    dst = os.path.join(pym, "dialects", "v20")
    os.makedirs(dst, exist_ok=True)
    for xml in glob.glob(os.path.join(mav, "message_definitions", "v1.0", "*.xml")):
        shutil.copy(xml, dst)
    try:
        import lxml  # noqa: F401
    except ImportError:
        print("error: lxml is missing (pip install lxml)", file=sys.stderr)
        return 1
    print(">> generating the ardupilotmega dialect")
    subprocess.run([sys.executable, "-c",
                    "from generator import mavgen, mavparse\n"
                    "assert mavgen.mavgen_python_dialect('ardupilotmega', mavparse.PROTOCOL_2_0)\n"
                    "print('ok')"], cwd=pym, check=True)
    sep = ";" if os.name == "nt" else ":"
    print(f"""
Done. In every shell and every script:

    PYTHONPATH={VENDOR}{sep}$PYTHONPATH     (PowerShell: $env:PYTHONPATH="{VENDOR}")
    MAVLINK20=1
    MAVLINK_DIALECT=ardupilotmega

MAVLINK20=1 is mandatory - without it set_dialect() looks in dialects/v10/ and regenerates
or fails. MAVLINK_DIALECT=ardupilotmega avoids generating the much larger "all" dialect.

Then, for example:

    python {os.path.join(VENDOR, 'pymavlink', 'tools', 'mavfft_isb.py')} flight.bin
    python -c "from pymavlink import mavextra; print(mavextra.expected_earth_field)"
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
