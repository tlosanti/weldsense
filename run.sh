#!/usr/bin/env bash
# =============================================================================
# WeldSense one-command launcher (macOS / Linux)
#
#   ./run.sh
#
# First run: creates a local Python virtual-env and installs dependencies.
# Every run: starts the host and opens the dashboard in your browser.
# The XIAO must be plugged in via a DATA-capable USB-C cable.
# =============================================================================
set -e
cd "$(dirname "$0")"

PY="${PYTHON:-python3}"
VENV=".venv"
URL="http://127.0.0.1:8765"

if ! command -v "$PY" >/dev/null 2>&1; then
  echo "ERROR: python3 not found. Install Python 3 (e.g. 'brew install python')."
  exit 1
fi

# ---- First-run setup (idempotent) ----
if [ ! -d "$VENV" ]; then
  echo "[setup] creating virtual environment..."
  "$PY" -m venv "$VENV"
fi

# Install / update deps only when requirements change (marker file).
REQ="host/requirements.txt"
STAMP="$VENV/.deps-installed"
if [ ! -f "$STAMP" ] || [ "$REQ" -nt "$STAMP" ]; then
  echo "[setup] installing dependencies..."
  "$VENV/bin/pip" install --quiet --upgrade pip
  "$VENV/bin/pip" install --quiet -r "$REQ"
  touch "$STAMP"
fi

# ---- Open the dashboard shortly after the server starts ----
( sleep 1.5
  if command -v open  >/dev/null 2>&1; then open "$URL"
  elif command -v xdg-open >/dev/null 2>&1; then xdg-open "$URL"
  fi ) >/dev/null 2>&1 &

echo "[run] starting WeldSense host -> $URL   (Ctrl+C to stop)"
exec "$VENV/bin/python" host/weldsense_host.py "$@"
