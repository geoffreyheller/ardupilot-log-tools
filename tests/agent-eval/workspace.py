"""Make a clean workspace for one agent's acceptance run.

    python tests/agent-eval/workspace.py DIR

DIR gets the fixture, a PX4 ULog header (for the out-of-scope case), and the skill at
project level in `.agents/skills/` and `.claude/skills/`, installed by `alog install-skill`
from this checkout, so between them every agent in the README table finds it without a
user-level install. Nothing from the repository itself (no AGENTS.md, CLAUDE.md or
docs) is copied: the run must show what the skill alone does. Start the agent in DIR.
"""
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
from cases import FIXTURE, ULOG  # noqa: E402

# The first 16 bytes of a PX4 ULog file: magic, version 1, a zero timestamp.
ULOG_HEADER = b"ULog\x01\x12\x35\x01" + bytes(8)


def make(dest):
    if os.path.exists(dest) and os.listdir(dest):
        raise SystemExit(f"{dest} is not empty; give a new directory")
    os.makedirs(dest, exist_ok=True)
    shutil.copyfile(os.path.join(ROOT, "tests", "fixtures", FIXTURE), os.path.join(dest, FIXTURE))
    with open(os.path.join(dest, ULOG), "wb") as fh:
        fh.write(ULOG_HEADER)
    for skills in (".agents/skills", ".claude/skills"):
        subprocess.run([sys.executable, os.path.join(ROOT, "alog.py"), "install-skill",
                        "--dir", os.path.join(dest, skills)], check=True)
    print(f"workspace ready: {dest}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    make(os.path.abspath(sys.argv[1]))
