"""The agent skill (skills/ardupilot-log-analysis/) is current, valid and agent-neutral.

The skill's references/ and the generated regions of SKILL.md come from the canonical docs
through tools/build_skill.py; these tests fail when a doc changed and the skill was not
rebuilt, when SKILL.md breaks the Agent Skills frontmatter rules (https://agentskills.io),
or when vendor-specific wording gets in.

    python tests/test_skill.py
"""

import os
import re
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

try:
    import pytest                       # noqa: F401
except ImportError:
    import _shim as pytest              # noqa: F401

import build_skill                                   # noqa: E402
from dflog import __version__, docs                  # noqa: E402

SKILL = build_skill.SKILL


def _read(rel):
    with open(os.path.join(SKILL, rel), encoding="utf-8") as fh:
        return fh.read().replace("\r\n", "\n")


def _files():
    out = []
    for base, _, names in os.walk(SKILL):
        out += [os.path.relpath(os.path.join(base, n), SKILL) for n in names]
    return sorted(out)


def _frontmatter():
    """SKILL.md's frontmatter as {key: str}, with metadata.* flattened. A small reader for
    the subset this file uses (scalars, one folded `>-` block, one nested map), so the test
    needs no YAML library."""
    text = _read("SKILL.md")
    assert text.startswith("---\n"), "SKILL.md must open with a --- frontmatter block"
    block = text[4:text.index("\n---\n", 4)]
    out, key, folded, parent = {}, None, False, None
    for line in block.split("\n"):
        if folded and line.startswith("  "):
            out[key] = (out[key] + " " + line.strip()).strip()
            continue
        folded = False
        m = re.match(r"^(  )?([a-z_-]+):\s*(.*)$", line)
        assert m, f"frontmatter line not understood: {line!r}"
        nested, k, v = m.group(1), m.group(2), m.group(3)
        if nested:
            assert parent, line
            k = f"{parent}.{k}"
        elif v == "":
            parent = k
            continue
        else:
            parent = None
        if v == ">-":
            out[k], key, folded = "", k, True
        else:
            out[k] = v.strip('"')
    return out


def test_skill_is_current():
    stale = [rel for rel, text in build_skill.build().items()
             if not os.path.isfile(os.path.join(SKILL, rel)) or _read(rel) != text]
    extra = [f for f in os.listdir(os.path.join(SKILL, "references"))
             if os.path.join("references", f) not in build_skill.build()]
    assert not stale and not extra, f"stale {stale}, not generated {extra}: run python tools/build_skill.py"


def test_frontmatter_follows_the_agent_skills_spec():
    fm = _frontmatter()
    assert set(fm) <= {"name", "description", "license", "compatibility", "allowed-tools"} | {
        k for k in fm if k.startswith("metadata.")}, sorted(fm)
    name = fm["name"]
    assert re.fullmatch(r"[a-z0-9]+(-[a-z0-9]+)*", name) and len(name) <= 64, name
    assert name == os.path.basename(SKILL), "name must match the folder name"
    assert 1 <= len(fm["description"]) <= 1024, len(fm["description"])
    assert len(fm.get("compatibility", "")) <= 500
    assert fm["license"] == "MIT"
    assert fm["metadata.version"] == __version__, (fm["metadata.version"], __version__)


def test_description_names_what_it_covers_and_what_it_does_not():
    d = _frontmatter()["description"].lower()
    for word in ("ardupilot", "ardu", "dataflash", ".bin", "vibration", "notch", "motor", "gps",
                 "ekf", "current", "pid"):
        assert word in d, word
    assert "not for px4 .ulg" in d and ".tlog" in d


def test_skill_md_is_short_and_every_linked_reference_exists():
    text = _read("SKILL.md")
    assert len(text.split("\n")) < 300
    linked = set(re.findall(r"references/([a-z0-9-]+\.md)", text))
    assert linked == set(build_skill.REFERENCES), linked ^ set(build_skill.REFERENCES)


def test_references_link_only_to_files_the_skill_has():
    have = set(build_skill.REFERENCES)
    for rel in _files():
        if rel.endswith(".md"):
            for name in re.findall(r"`([a-z0-9-]+\.md)`", _read(rel)):
                assert name in have, f"{rel} links to {name}, which the skill does not carry"


def test_every_alog_docs_topic_the_skill_names_exists():
    topics, _ = docs.topics()
    for rel in _files():
        for topic in re.findall(r"alog docs ([a-z][a-z0-9-]*)", _read(rel)):
            assert topic in topics, f"{rel}: `alog docs {topic}` is not a topic"


def test_no_agent_specific_wording():
    vendors = re.compile(r"\b(claude|anthropic|copilot|openai|codex|cursor|gemini|chatgpt|gpt-\d)\b", re.I)
    slash = re.compile(r"(?:^|\s)/[a-z][a-z-]+\b(?!/)", re.M)
    tools = re.compile(r"\b(Bash|Read|Edit|Write|Grep|Glob|WebFetch|WebSearch|TodoWrite)\s+tool\b|mcp__")
    assert slash.search("then run /plugin install x") and not slash.search("<!--  -->a / b, c/d")
    for rel in _files():
        text = re.sub(r"<!--.*?-->", "", _read(rel), flags=re.S)    # the generated-region markers
        for pat, what in ((vendors, "vendor name"), (slash, "slash command"), (tools, "agent tool name")):
            m = pat.search(text)
            assert not m, f"{rel}: {what} {m.group(0).strip()!r}"


def test_run_scripts_pin_the_package_version_and_parse():
    sh, ps1 = _read("scripts/alog.sh"), _read("scripts/alog.ps1")
    ref = build_skill.git_ref(__version__)
    assert f'PIN="{__version__}"' in sh and f'REF="{ref}"' in sh
    assert f"$Pin = '{__version__}'" in ps1 and f"$Ref = '{ref}'" in ps1
    assert sh.startswith("#!/bin/sh\n")
    with open(os.path.join(ROOT, ".gitattributes"), encoding="utf-8") as fh:
        assert "*.sh text eol=lf" in fh.read()
    shell = shutil.which("sh") or shutil.which("bash")
    if shell:
        assert subprocess.call([shell, "-n", os.path.join(SKILL, "scripts", "alog.sh")]) == 0


def test_git_ref_is_the_tag_for_a_release_and_main_for_a_dev_build():
    assert build_skill.git_ref("2.1.0") == "v2.1.0"
    assert build_skill.git_ref("2.1.0.dev0") == "main"
    assert build_skill.git_ref("2.1.0rc1") == "main"


def test_strip_fails_loudly_when_the_heading_is_gone():
    try:
        build_skill.strip_sections("# T\n\n## 1. Kept\n", ["## 5. Changing the tools."], "RULES.md")
    except ValueError as exc:
        assert "Changing the tools" in str(exc)
    else:
        raise AssertionError("a missing strip heading was ignored")


def test_rules_reference_has_no_repository_development_section():
    rules = _read("references/rules.md")
    assert "## 4. Analysis discipline" in rules and "Changing the tools" not in rules


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
