"""A 40-line stand-in for the two pytest features these tests use.

pytest is not installable in the Claude cloud sandbox (PyPI is blocked), and vendoring it
for `approx` and `skip` would be absurd. `python3 tests/test_toolkit.py` uses this; if real
pytest is available it is used instead and this file is ignored.
"""


class Skipped(Exception):
    pass


class _Approx:
    def __init__(self, expected, rel=None, abs=None):
        self.expected, self.rel, self.abs = expected, rel, abs

    def __eq__(self, actual):
        if self.abs is not None and abs(actual - self.expected) <= self.abs:
            return True
        rel = self.rel if self.rel is not None else 1e-6
        return abs(actual - self.expected) <= rel * max(abs(self.expected), abs(actual), 1e-12)

    def __repr__(self):
        tol = f"abs={self.abs}" if self.abs is not None else f"rel={self.rel or 1e-6}"
        return f"approx({self.expected}, {tol})"


def approx(expected, rel=None, abs=None):
    return _Approx(expected, rel, abs)


def skip(reason=""):
    raise Skipped(reason)


def run(module):
    """Run every test_* callable in `module`. Returns the number of failures."""
    names = [n for n in dir(module) if n.startswith("test_")]
    failed = skipped = passed = 0
    for n in sorted(names):
        try:
            getattr(module, n)()
        except Skipped as exc:
            print(f"SKIP {n}: {exc}")
            skipped += 1
        except AssertionError as exc:
            print(f"FAIL {n}: {exc or '(no message)'}")
            failed += 1
        except Exception as exc:
            print(f"ERROR {n}: {type(exc).__name__}: {exc}")
            failed += 1
        else:
            print(f"ok   {n}")
            passed += 1
    print(f"\n{passed} passed, {failed} failed, {skipped} skipped")
    return failed
