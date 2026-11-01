"""Local CI: run the whole test suite under every supported Python, on this machine.

    python tools/ci_local.py                  # Python 3.9 and 3.12
    python tools/ci_local.py --python 3.11    # one version (repeatable)

GitHub Actions does not run for this repository, so this is the check to run before
merging. For each Python version it creates a clean, throwaway environment from
`requirements.txt` with uv (`uv run --no-project --isolated`, nothing is written to the
repo), then runs `tests/run_all.py` and the `alog.py --help` smoke test.

It also builds the package the way a release would (`uv build`: the sdist, then the wheel
from the sdist) and, for each Python version, installs that wheel into another throwaway
environment and runs it from a temporary directory outside the repo: `alog --version`,
`alog docs` and `alog install-skill` (which must use the copies inside the wheel, not the
repo), and `alog info` / `alog all --json` on the committed fixture with the cache in a
temporary directory. An
editable install or a clone hides missing package data; this does not. Exit code is 0
when every step passed, 1 otherwise, 3 when uv is not available.

It covers the operating system it runs on and nothing else. A pass on Windows says
nothing about Linux or macOS; the summary names the platform so a report cannot imply
otherwise. Tests that need the reference logs skip unless LOG_DIR is set.
"""
import argparse
import glob
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHONS = ["3.9", "3.12"]           # the floor and the newest tested; pyproject requires >=3.9
STEPS = [
    ("tests", ["tests/run_all.py"]),
    ("alog --help", ["alog.py", "--help"]),
]
FIXTURE = os.path.join(ROOT, "tests", "fixtures", "largeprop-quad.bin")


def _installed_steps(cache):
    """(label, alog arguments, accept(returncode, stdout) -> bool) run against the wheel."""
    def docs_are_installed(rc, out):
        return rc == 0 and json.loads(out)["where"] == "installed" and len(json.loads(out)["topics"]) > 4

    def report(rc, out):            # the fixture's own verdict is FAIL (exit 2); 3 is an input error
        return rc in (0, 1, 2) and json.loads(out)["command"] == "all"
    return [
        ("wheel: --version", ["--version"], lambda rc, out: rc == 0 and "ardupilot-log-tools" in out),
        ("wheel: docs", ["--json", "docs"], docs_are_installed),
        ("wheel: docs topic", ["docs", "pitfalls"], lambda rc, out: rc == 0 and out.startswith("# Pitfalls")),
        ("wheel: install-skill", ["--json", "install-skill", "--dir", os.path.join(os.path.dirname(cache), "skills")],
         lambda rc, out: rc == 0 and json.loads(out)["source"] == "installed" and "SKILL.md" in json.loads(out)["files"]),
        ("wheel: info", ["--cache-dir", cache, "info", FIXTURE], lambda rc, out: rc == 0),
        ("wheel: all --json", ["--cache-dir", cache, "--json", "all", FIXTURE], report),
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
    work = tempfile.mkdtemp(prefix="alog-ci-")
    print("\n=== build: sdist, then wheel from the sdist", flush=True)
    t0 = time.time()
    # Build from the files a fresh clone would have, not the working tree: a stale, ignored
    # *.egg-info/SOURCES.txt there feeds setuptools its old file list and hides a file
    # missing from MANIFEST.in (it did, for GUIDE.md).
    tree = os.path.join(work, "src")
    listed = subprocess.run(["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
                            cwd=ROOT, capture_output=True, check=True).stdout.decode("utf-8").split("\0")
    for rel in filter(None, listed):
        if os.path.isfile(os.path.join(ROOT, rel)):          # a tracked file deleted in the tree
            os.makedirs(os.path.dirname(os.path.join(tree, rel)), exist_ok=True)
            shutil.copy2(os.path.join(ROOT, rel), os.path.join(tree, rel))
    rc = subprocess.call(uv + ["build", "--out-dir", os.path.join(work, "dist")], cwd=tree)
    wheels = glob.glob(os.path.join(work, "dist", "*.whl"))
    results.append(("-", "build", 0 if rc == 0 and len(wheels) == 1 else 1, time.time() - t0))
    for py in args.python or PYTHONS:
        if wheels:
            outside = tempfile.mkdtemp(prefix="alog-ci-run-", dir=work)
            for label, alog_args, accept in _installed_steps(os.path.join(outside, "cache")):
                t0 = time.time()
                proc = subprocess.run(uv + ["run", "--no-project", "--isolated", "--python", py,
                                            "--with", wheels[0], "alog"] + alog_args,
                                      cwd=outside, capture_output=True, text=True, encoding="utf-8")
                try:
                    ok = accept(proc.returncode, proc.stdout)
                except (ValueError, KeyError):
                    ok = False
                if not ok:
                    print(f"\n=== Python {py}: {label} FAILED (exit {proc.returncode})\n"
                          f"{proc.stdout[-2000:]}{proc.stderr[-2000:]}", flush=True)
                results.append((py, label, 0 if ok else 1, time.time() - t0))
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
        print(f"{'ok  ' if rc == 0 else 'FAIL'} Python {py:5s} {label:18s} {dt:6.1f} s")
    shutil.rmtree(work, ignore_errors=True)
    failed = [r for r in results if r[2] != 0]
    print(f"\n{len(results) - len(failed)} of {len(results)} steps passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
