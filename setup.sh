#!/usr/bin/env bash
set -euo pipefail

VENV_DIR=".venv"
REQ_FILE="requirements.txt"

if command -v python3 >/dev/null 2>&1; then
  PY=python3
elif command -v python >/dev/null 2>&1; then
  PY=python
else
  echo "Python not found. Install Python 3 and rerun." >&2
  exit 1
fi

if [ ! -f "$REQ_FILE" ]; then
  cat > "$REQ_FILE" <<'EOF'
# Requirements for this project
# Add packages below, one per line. Example:
# requests==2.31.0
EOF
  echo "Created $REQ_FILE (template)."
fi

if [ -d "$VENV_DIR" ]; then
  echo "Virtualenv already exists at $VENV_DIR"
else
  echo "Creating virtual environment at $VENV_DIR..."
  "$PY" -m venv "$VENV_DIR"
fi

# Simple POSIX-style venv: assume bin/ layout (works on Linux, macOS, WSL, Git Bash)
VENV_PY="$VENV_DIR/bin/python"
if [ ! -x "$VENV_PY" ]; then
  echo "Virtualenv python not found at $VENV_PY" >&2
  exit 1
fi

echo "Installing requirements..."
"$VENV_PY" -m pip install -r "$REQ_FILE"

echo
echo "Done. Activate the venv with:"
echo "  source $VENV_DIR/bin/activate"
