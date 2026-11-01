#!/bin/sh
# Run the `alog` this skill was written for, with any arguments: scripts/alog.sh info flight.bin
#
# Order: an installed `alog` whose --version is the pinned one; else uvx (or `uv tool run`);
# else pipx; else an installed `alog` of another version, with a warning; else print how to
# install it and exit 3 (the CLI's own "could not run" code). PIN and REF are set by
# tools/build_skill.py from the package version: REF is the release tag, or main for a
# development version.
PIN="2.1.0"
REF="v2.1.0"
URL="https://github.com/geoffreyheller/ardupilot-log-tools"
SPEC="git+$URL@$REF"

if command -v alog >/dev/null 2>&1 && [ "$(alog --version 2>/dev/null)" = "ardupilot-log-tools $PIN" ]; then
    exec alog "$@"
fi
if command -v uvx >/dev/null 2>&1; then
    exec uvx --from "$SPEC" alog "$@"
fi
if command -v uv >/dev/null 2>&1; then
    exec uv tool run --from "$SPEC" alog "$@"
fi
if command -v pipx >/dev/null 2>&1; then
    exec pipx run --spec "$SPEC" alog "$@"
fi
if command -v alog >/dev/null 2>&1; then
    echo "warning: using installed $(alog --version 2>/dev/null); this skill was written for $PIN" >&2
    exec alog "$@"
fi
echo "alog $PIN is not installed, and neither uv nor pipx is available to run it." >&2
echo "Install it with:  pip install \"$SPEC\"   (Python 3.9+)" >&2
exit 3
