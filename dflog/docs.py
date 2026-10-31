"""The project's documentation, readable from an installed package: `alog docs`.

The repo's markdown is canonical and is never committed twice. Building a wheel
(`setup.py`, `build_py`) copies every file `sources()` names into `dflog/docs/<topic>.md`;
in a clone or an editable install there is no such directory and the same files are read
in place. Either way `alog docs pitfalls` prints the pitfalls that match the code the
reader is running.

Stdlib only, and no relative imports: `setup.py` loads this file with `runpy` before any
dependency is installed.
"""
import glob
import os

# topic -> path relative to the repository root. Every reference/*.md is a topic named
# after its file, so a new reference document ships without being listed here.
ROOT_TOPICS = {
    "rules": "RULES.md",
    "guide": "GUIDE.md",
    "skills": "SKILLS.md",
    "report-template": "templates/log-analysis-template.md",
}
REFERENCE_GLOB = "reference/*.md"

_PKG = os.path.dirname(os.path.abspath(__file__))
INSTALLED_DIR = os.path.join(_PKG, "docs")
REPO_ROOT = os.path.dirname(_PKG)


def sources(root=REPO_ROOT):
    """{topic: absolute path} of the canonical files under a repository root.

    Raises FileNotFoundError naming the file when a root topic is missing, so a build
    from an incomplete source tree fails instead of shipping a wheel without its docs."""
    out = {}
    for topic, rel in ROOT_TOPICS.items():
        path = os.path.join(root, rel)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"documentation source missing: {rel} (under {root})")
        out[topic] = path
    for path in sorted(glob.glob(os.path.join(root, REFERENCE_GLOB))):
        topic = os.path.splitext(os.path.basename(path))[0]
        if topic in out:
            raise ValueError(f"reference/{topic}.md collides with the root topic {topic!r}")
        out[topic] = path
    if len(out) == len(ROOT_TOPICS):
        raise FileNotFoundError(f"no {REFERENCE_GLOB} under {root}")
    return out


def topics():
    """({topic: path}, where) for the docs this installation can show.

    `where` is "installed" (the copy inside the wheel) or "repository" (a clone or an
    editable install, read in place)."""
    if os.path.isdir(INSTALLED_DIR):
        found = {os.path.splitext(f)[0]: os.path.join(INSTALLED_DIR, f)
                 for f in sorted(os.listdir(INSTALLED_DIR)) if f.endswith(".md")}
        if found:
            return found, "installed"
    return sources(REPO_ROOT), "repository"


def read(topic):
    """The text of one topic. KeyError names the topics that exist."""
    found, _ = topics()
    if topic not in found:
        raise KeyError(f"no documentation topic {topic!r}; topics: {', '.join(sorted(found))}")
    with open(found[topic], encoding="utf-8") as fh:
        return fh.read()


def title(path):
    """The first markdown heading of a file, or its name."""
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if line.startswith("#"):
                return line.lstrip("#").strip()
    return os.path.basename(path)
