"""Run every tests/test_*.py file, each in its own process, and summarise.

    python tests/run_all.py                 # every file
    python tests/run_all.py cli flights     # only test_cli.py and test_flights.py

Files are found by glob, so a new test file runs here (and in CI) without being listed
anywhere. Each runs standalone through the `_shim` runner, exactly as
`python tests/test_x.py` would, so pytest is not needed. Exit code is 0 when every file
passed, 1 otherwise. Tests that need the reference logs skip unless LOG_DIR is set.
"""
import glob
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def main(argv):
    files = sorted(glob.glob(os.path.join(HERE, "test_*.py")))
    if argv:
        wanted = {(a if a.startswith("test_") else "test_" + a).removesuffix(".py") + ".py"
                  for a in argv}
        files = [f for f in files if os.path.basename(f) in wanted]
        missing = wanted - {os.path.basename(f) for f in files}
        if missing:
            print(f"no such test file: {', '.join(sorted(missing))}")
            return 1
    results = []
    for path in files:
        name = os.path.basename(path)
        print(f"=== {name}", flush=True)
        t0 = time.time()
        with open(path, encoding="utf-8") as fh:
            standalone = "_shim.run(" in fh.read()
        if not standalone:
            # Without the `_shim.run` main block the file exits 0 having run nothing.
            print(f"FAIL {name}: no `sys.exit(_shim.run(...))` main block, so no test in it would run")
            rc = 1
        else:
            rc = subprocess.call([sys.executable, path])
        results.append((name, rc, time.time() - t0))
    print("\n=== summary")
    for name, rc, dt in results:
        print(f"{'ok  ' if rc == 0 else 'FAIL'} {name:32s} {dt:6.1f} s")
    failed = [name for name, rc, _ in results if rc != 0]
    print(f"\n{len(results) - len(failed)} of {len(results)} files passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
