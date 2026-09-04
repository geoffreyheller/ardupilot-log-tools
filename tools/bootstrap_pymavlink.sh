#!/usr/bin/env bash
# Vendor pymavlink from GitHub into ./vendor/, with MAVLink dialects generated.
#
# You do NOT need this for anything in alog.py. Run it when you want:
#   - mavextra.py            WMM expected earth field, magfit, stateful filters
#   - tools/mavfft_isb.py    the reference batch-IMU FFT tool
#   - MAVExplorer            interactive graphing (needs MAVProxy too)
#
# Why a script: PyPI is blocked in the Claude cloud sandbox, and pymavlink's repo ships
# no generated dialect modules - `dialects/.gitignore` excludes *.py, and
# message_definitions/ lives in a different repo (ArduPilot/mavlink). DFReader.py imports
# mavutil, which calls set_dialect() at import time and dies without them. So a plain
# clone is not importable.
#
# Requires: git, python3, lxml.

set -euo pipefail
cd "$(dirname "$0")/.."
VENDOR="$PWD/vendor"
mkdir -p "$VENDOR"

if [ ! -d "$VENDOR/pymavlink" ]; then
  echo ">> cloning pymavlink"
  git clone --depth 1 -q https://github.com/ArduPilot/pymavlink.git "$VENDOR/pymavlink"
fi

if [ ! -d "$VENDOR/mavlink" ]; then
  echo ">> sparse-cloning ArduPilot/mavlink for the message definitions"
  git clone --depth 1 --filter=blob:none --sparse -q \
      https://github.com/ArduPilot/mavlink.git "$VENDOR/mavlink"
  git -C "$VENDOR/mavlink" sparse-checkout set message_definitions
fi

# mavgen resolves <include> relative to the target directory, so copy ALL the XMLs,
# not just ardupilotmega.xml.
cp "$VENDOR"/mavlink/message_definitions/v1.0/*.xml "$VENDOR/pymavlink/dialects/v20/"

if ! python3 -c "import lxml" 2>/dev/null; then
  echo "!! lxml is missing and PyPI may be blocked. Try: pip install lxml --break-system-packages"
  exit 1
fi

echo ">> generating the ardupilotmega dialect"
( cd "$VENDOR/pymavlink" && python3 -c "
from generator import mavgen, mavparse
assert mavgen.mavgen_python_dialect('ardupilotmega', mavparse.PROTOCOL_2_0)
print('ok')
" )

cat <<'NOTE'

Done. To use it, in every shell and every script:

    export PYTHONPATH="<this folder>/vendor:$PYTHONPATH"
    export MAVLINK20=1
    export MAVLINK_DIALECT=ardupilotmega

MAVLINK20=1 is mandatory - without it set_dialect() looks in dialects/v10/ and
regenerates or fails. MAVLINK_DIALECT=ardupilotmega avoids generating the much larger
"all" dialect.

Then, for example:

    python3 vendor/pymavlink/tools/mavfft_isb.py "<log>.bin"
    python3 -c "from pymavlink import mavextra; print(mavextra.expected_earth_field)"

The dfindexer/ Cython accelerator is not needed; DFReader degrades gracefully without it.
NOTE
