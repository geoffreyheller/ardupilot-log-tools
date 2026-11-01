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
    stale += [rel for rel, text in build_skill.adapters().items()
              if open(os.path.join(ROOT, rel), encoding="utf-8").read().replace("\r\n", "\n") != text]
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


# --------------------------------------------- the skill quotes the real CLI (S3)

def _quoted_commands(text):
    """Every `alog ...` the text quotes: code-block lines and inline code spans, up to a
    comment, a pipe or a redirect, as token lists after `alog`."""
    out = []
    for m in re.finditer(r"(?:^[ \t]*|`)alog((?: [^`\n]*)?)", text, re.M):
        toks = []
        for tok in m.group(1).split():
            if tok.startswith("#") or tok in ("|", ">", ">>", "&&"):
                break
            toks.append(tok)
        out.append(toks)
    return out


def _options(parser):
    """{option string: takes a value} for one argparse parser."""
    return {s: a.nargs != 0 for a in parser._actions for s in a.option_strings}


def _check_command(toks, root, subs):
    """None if `alog <toks>` uses real commands and flags, else why not."""
    from dflog.flight import WINDOW_METHODS
    glob = _options(root)
    placeholder = re.compile(r"^(…|\.\.\.|[A-Z][A-Z0-9_]*|<.*>|\[.*\]?|.*\|.*)$")
    i = 0
    while i < len(toks) and toks[i].startswith("-"):           # global flags
        flag = toks[i].split("=")[0]
        if flag not in glob:
            return f"{flag} is not a global option"
        i += 2 if glob[flag] and "=" not in toks[i] else 1
    if i == len(toks) or placeholder.match(toks[i]):
        return None
    cmd = toks[i]
    if cmd not in subs:
        return f"no subcommand {cmd!r}"
    opts = _options(subs[cmd])
    rest = toks[i + 1:]
    for j, tok in enumerate(rest):
        flag = tok.strip("[]`,.;:)").split("=")[0]
        if not flag.startswith("--"):
            continue
        if flag not in opts:
            where = " (a global option: it goes before the subcommand)" if flag in glob else ""
            return f"{cmd} has no option {flag}{where}"
        if flag == "--window" and j + 1 < len(rest):
            value = rest[j + 1].strip("`,.;:)")
            ok = (value in WINDOW_METHODS or placeholder.match(value)
                  or re.fullmatch(r"hover:\d+|[\d.]+:[\d.]+", value))
            if not ok:
                return f"--window {value} is not a window method"
    return None


def test_every_quoted_alog_command_is_real():
    from dflog.cli import build_parser
    root = build_parser()
    subs = next(a for a in root._actions if a.__class__.__name__ == "_SubParsersAction").choices
    docs_outside = [os.path.join(ROOT, "README.md"), os.path.join(ROOT, "AGENTS.md")]
    bad, seen = [], 0
    for path in [os.path.join(SKILL, rel) for rel in _files() if rel.endswith(".md")] + docs_outside:
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        for toks in _quoted_commands(text):
            seen += 1
            why = _check_command(toks, root, subs)
            if why:
                bad.append(f"{os.path.relpath(path, ROOT)}: `alog {' '.join(toks)}`: {why}")
    assert seen > 100, seen                 # the extractor still finds the commands
    assert not bad, "\n".join(bad)


def test_the_command_check_catches_drift():
    from dflog.cli import build_parser
    root = build_parser()
    subs = next(a for a in root._actions if a.__class__.__name__ == "_SubParsersAction").choices
    assert _check_command(["info", "flight.bin"], root, subs) is None
    assert _check_command(["--cache-dir", "DIR", "info", "x.bin"], root, subs) is None
    assert _check_command(["all", "x.bin", "--window", "hover:2", "--flight", "all"], root, subs) is None
    assert "no subcommand" in _check_command(["motor", "x.bin"], root, subs)
    assert "no option --windwo" in _check_command(["motors", "x.bin", "--windwo", "rpm"], root, subs)
    assert "goes before" in _check_command(["info", "x.bin", "--no-cache"], root, subs)
    assert "not a window method" in _check_command(["vibe", "x.bin", "--window", "hovr"], root, subs)


def _exit_code_claims(text):
    """{code: meaning} from 'Exit codes: 0 pass, 1 warn, ...' sentences."""
    claims = {}
    for m in re.finditer(r"Exit codes?: ((?:\d [^,.]+(?:, |\.))+)", text):
        for code, meaning in re.findall(r"(\d) ([^,.]+)", m.group(1)):
            claims[int(code)] = meaning.strip()
    return claims


def test_quoted_exit_codes_match_the_schema():
    from dflog.cli import main
    import io
    import json
    from contextlib import redirect_stdout
    buf = io.StringIO()
    with redirect_stdout(buf):
        main(["--json", "schema"])
    schema = {int(k): v for k, v in json.loads(buf.getvalue())["exit_codes"].items()}
    word = {0: "pass", 1: "warn", 2: "fail", 3: "input"}
    for rel in ("SKILL.md", "references/recipes.md"):
        claims = _exit_code_claims(_read(rel))
        assert set(claims) == set(schema), (rel, claims, schema)
        for code, meaning in claims.items():
            assert word[code] in meaning.lower() and word[code] in schema[code].lower(), (rel, code, meaning)


def _alog_json(*argv):
    from dflog.cli import main
    import io
    import json
    from contextlib import redirect_stdout
    buf = io.StringIO()
    with redirect_stdout(buf):
        main(["--no-cache", "--json"] + list(argv))
    return json.loads(buf.getvalue())


def test_documented_json_keys_are_the_ones_alog_emits():
    import tempfile
    from synthlog import standard_log
    import test_cli
    tmp = tempfile.mkdtemp(prefix="dflog-skill-")
    one = standard_log(n=200).write(os.path.join(tmp, "one.bin"))
    two = test_cli._two_flight_log(os.path.join(tmp, "two.bin"))
    contract = _read("references/json-output.md")
    blocks = re.findall(r"```json\n(.*?)```", contract, re.S)
    envelope = set(re.findall(r'^  "([a-z_]+)":', blocks[0], re.M))
    report = set(re.findall(r'^"([a-z_]+)":', blocks[1], re.M))
    got = set(_alog_json("all", one))
    assert envelope | report == got, f"documented {sorted(envelope | report)} != emitted {sorted(got)}"
    # `--flight all`: the third example lists what replaces `sections`
    flight_all = set(re.findall(r'^"([a-z_]+)":', blocks[2], re.M))
    got = set(_alog_json("all", two, "--flight", "all"))
    assert envelope | flight_all == got, f"documented {sorted(envelope | flight_all)} != emitted {sorted(got)}"
    assert "sections" not in got
    assert "`per_flight`" in _read("SKILL.md") and "`sections`" in _read("SKILL.md")


def test_rules_reference_has_no_repository_development_section():
    rules = _read("references/rules.md")
    assert "## 4. Analysis discipline" in rules and "Changing the tools" not in rules


if __name__ == "__main__":
    import _shim
    sys.exit(_shim.run(sys.modules[__name__]))
