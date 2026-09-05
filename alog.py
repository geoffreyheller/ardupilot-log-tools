#!/usr/bin/env python3
"""alog - ArduPilot dataflash log analysis CLI. Thin launcher for dflog.cli.

    python alog.py info flight.bin
    python alog.py all  flight.bin --json

Works the same on Windows, Linux and macOS. `pip install -e .` also installs an
`alog` console script so the `python alog.py` prefix is not needed.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dflog.cli import entry as main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
