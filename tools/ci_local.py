"""Local CI: run the whole test suite under every supported Python, on this machine.

    python tools/ci_local.py                  # Python 3.9 and 3.12
    python tools/ci_local.py --python 3.11    # one version (repeatable)

GitHub Actions does not run for this repository, so this is the check to run before
merging. For each Python version it creates a clean, throwaway environment from
`requirements.txt` with uv (`uv run --no-project --isolated`, nothing is written to the
repo), then runs `tests/run_all.py` and the `alog.py --help` smoke test. Exit code is 0
when every version passed, 1 otherwise, 3 when uv is not available.

It covers the operating system it runs on and nothing else. A pass on Windows says
nothing about Linux or macOS; the summary names the platform so a report cannot imply
otherwise. Tests that need the reference logs skip unless LOG_DIR is set.
"""
import argparse
import os
import platform
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHONS = ["3.9", "3.12"]           # the floor and the newest tested; pyproject requires >=3.9
STEPS = [
    ("tests", ["tests/run_all.py"]),
    ("alog --help", ["alog.py", "--help"]),
]


def _uv():
    """The uv command: `uv` on PATH, else `python -m uv` (pip install uv), else None."""
    if shutil.which("uv"):
        return ["uv"]
    if subprocess.call([sys.executable, "-m", "uv", "--version"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
        return [sys.executable, "-m", "uv"]
    return None


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--python", action="append", metavar="VERSION",
                    help=f"Python version to test (repeatable; default {' and '.join(PYTHONS)})")
    args = ap.parse_args(argv)
    uv = _uv()
    if uv is None:
        print("uv is not installed: `pip install uv` (or see https://docs.astral.sh/uv/), then rerun")
        return 3
    results = []
    for py in args.python or PYTHONS:
        for label, cmd in STEPS:
            print(f"\n=== Python {py}: {label}", flush=True)
            t0 = time.time()
            rc = subprocess.call(uv + ["run", "--no-project", "--isolated", "--python", py,
                                       "--with-requirements", "requirements.txt", "python"] + cmd,
                                 cwd=ROOT, stdout=None if label == "tests" else subprocess.DEVNULL)
            results.append((py, label, rc, time.time() - t0))
    print(f"\n=== local CI summary ({platform.system()} {platform.machine()}; "
          f"other operating systems are not covered)")
    for py, label, rc, dt in results:
        print(f"{'ok  ' if rc == 0 else 'FAIL'} Python {py:5s} {label:12s} {dt:6.1f} s")
    failed = [r for r in results if r[2] != 0]
    print(f"\n{len(results) - len(failed)} of {len(results)} steps passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
