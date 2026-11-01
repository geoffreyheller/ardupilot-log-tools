"""Grade agent transcripts against cases.py.

    python tests/agent-eval/grade.py 1-triage transcript.md
    python tests/agent-eval/grade.py --dir runs/claude-code     # every <case-id>.md there

A transcript is markdown with two sections; everything outside them is ignored:

    ## Commands
    alog info largeprop-quad.bin
    alog all largeprop-quad.bin --json

    ## Answer
    The log is intact...

Exit code 0 when every automatic criterion passed, 1 otherwise. Manual criteria are
printed with a `?` for a person to decide; they never change the exit code.
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cases import CASES  # noqa: E402

# the first alog subcommand on a command line, past the global options and the skill's
# run scripts (scripts/alog.sh, alog.ps1)
_ALOG = re.compile(r"\balog(?:\.sh|\.ps1)?['\"]?\s+"
                   r"(?:(?:--json|--no-cache|--strict|--verbose)\s+|--cache-dir[ =]\S+\s+)*"
                   r"([a-z][a-z-]*)")


def sections(text):
    """{"commands": str, "answer": str} from a transcript. The answer runs from its heading
    to the end of the file: an agent's answer has markdown headings of its own."""
    answer = re.search(r"^## Answer[ \t]*\n", text, re.M)
    commands = re.search(r"^## Commands[ \t]*\n", text, re.M)
    end = answer.start() if answer and commands and answer.start() > commands.end() else len(text)
    return {"commands": text[commands.end():end] if commands else "",
            "answer": text[answer.end():] if answer else ""}


_ASSIGN = re.compile(r"\b([A-Za-z_]\w*)=([\"']?)([^\s;\"']+)\2")


def _expand(line):
    """Substitute this line's own simple variables, so `S=.../alog.sh; bash $S info` reads
    as the alog.sh invocation it is."""
    for name, _, value in _ASSIGN.findall(line):
        line = re.sub(r"\$\{?%s\}?(?!\w)" % re.escape(name), lambda m: value, line)
    return line


def first_subcommand(commands):
    for line in commands.splitlines():
        m = _ALOG.search(_expand(line))
        if m:
            return m.group(1)
    return None


def grade(case, text):
    """[(criterion, verdict)] with verdict True / False, or None for a manual one."""
    parts = sections(text)
    out = []
    for name, where, kind, pattern in case["criteria"]:
        body = parts[where]
        if kind == "manual":
            verdict = None
        elif kind == "first":
            verdict = first_subcommand(body) == pattern
        else:
            found = re.search(pattern, body, re.I) is not None
            verdict = found if kind == "present" else not found
        out.append((name, verdict))
    return out


def _report(case_id, path):
    case = next(c for c in CASES if c["id"] == case_id)
    with open(path, encoding="utf-8") as fh:
        results = grade(case, fh.read())
    print(f"## {case_id}  ({os.path.basename(path)})")
    for (name, verdict), crit in zip(results, case["criteria"]):
        mark = {True: "pass", False: "FAIL", None: "?   "}[verdict]
        extra = f"  [{crit[3]}]" if verdict is None else ""
        print(f"  {mark} {name}{extra}")
    return all(v is not False for _, v in results)


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("case", nargs="?", help="case id, e.g. 1-triage")
    ap.add_argument("transcript", nargs="?")
    ap.add_argument("--dir", help="grade <case-id>.md for every case found in DIR")
    args = ap.parse_args(argv)
    ids = [c["id"] for c in CASES]
    if args.dir:
        pairs = [(i, os.path.join(args.dir, i + ".md")) for i in ids if os.path.isfile(os.path.join(args.dir, i + ".md"))]
        if not pairs:
            print(f"no transcripts named <case-id>.md in {args.dir}; case ids: {', '.join(ids)}")
            return 1
    elif args.case in ids and args.transcript:
        pairs = [(args.case, args.transcript)]
    else:
        ap.error(f"give a case id ({', '.join(ids)}) and a transcript, or --dir")
    ok = [_report(i, p) for i, p in pairs]
    print(f"\n{sum(ok)} of {len(ok)} cases passed every automatic criterion")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
