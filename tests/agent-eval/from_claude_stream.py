"""Turn `claude -p ... --output-format stream-json --verbose` output into a transcript.

    claude -p "<prompt>" --output-format stream-json --verbose > run.jsonl
    python tests/agent-eval/from_claude_stream.py run.jsonl > 1-triage.md

Writes the `## Commands` (every shell command the agent ran, in order) and `## Answer`
(the final result) sections grade.py reads. Other agents: copy the same two things from
their own transcript by hand.
"""
import json
import sys


def convert(lines):
    commands, answer = [], ""
    for line in lines:
        line = line.strip()
        if not line.startswith("{"):
            continue
        event = json.loads(line)
        if event.get("type") == "assistant":
            for block in event.get("message", {}).get("content", []):
                if block.get("type") == "tool_use" and "command" in block.get("input", {}):
                    commands.append(" ".join(block["input"]["command"].split()))
        elif event.get("type") == "result":
            answer = event.get("result", "") or ""
    return "## Commands\n" + "".join(c + "\n" for c in commands) + "\n## Answer\n" + answer + "\n"


if __name__ == "__main__":
    with open(sys.argv[1], encoding="utf-8") as fh:
        sys.stdout.write(convert(fh))
