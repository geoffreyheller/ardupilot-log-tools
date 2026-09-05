#!/usr/bin/env python3
"""Notch verification chart. Thin launcher for dflog.plot_notch.

    python plot_notch.py flight.bin -o notch-verification-YYYY-MM-DD.png
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dflog.plot_notch import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
