#!/bin/bash
# MONOMOD GUI Runner
# Sets up virtual environment, installs dependencies, installs the monomod
# driver in editable mode, and runs the GUI.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"
REQUIREMENTS="$SCRIPT_DIR/requirements.txt"
MONOMOD_DRIVER="$SCRIPT_DIR/../driver"

echo "=== MONOMOD GUI Runner ==="

# Create (or repair) the virtual environment.
# Detect a partial/broken .venv (e.g. left behind by a failed creation) by
# checking for the activate script rather than just the directory.
if [ ! -f "$VENV_DIR/bin/activate" ]; then
    if [ -d "$VENV_DIR" ]; then
        echo "Existing .venv is incomplete — recreating..."
        rm -rf "$VENV_DIR"
    fi
    echo "Creating virtual environment at $VENV_DIR..."
    if ! python3 -m venv "$VENV_DIR"; then
        echo "ERROR: could not create the virtual environment." >&2
        echo "On Debian/Ubuntu, install the venv module first:" >&2
        echo "    sudo apt install python3-venv" >&2
        exit 1
    fi
    echo "Virtual environment created at $VENV_DIR"
fi

# Activate virtual environment
echo "Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Check if requirements need to be installed
MARKER_FILE="$VENV_DIR/.requirements_installed"
REQUIREMENTS_HASH=$(md5sum "$REQUIREMENTS" 2>/dev/null | cut -d' ' -f1 || echo "none")

if [ ! -f "$MARKER_FILE" ] || [ "$(cat "$MARKER_FILE" 2>/dev/null)" != "$REQUIREMENTS_HASH" ]; then
    echo "Installing/updating dependencies..."
    pip install --upgrade pip -q
    pip install -r "$REQUIREMENTS" -q

    # Install monomod driver in editable mode
    if [ -d "$MONOMOD_DRIVER" ]; then
        echo "Installing monomod driver (editable)..."
        pip install -e "$MONOMOD_DRIVER" -q
    else
        echo "WARNING: monomod driver not found at $MONOMOD_DRIVER"
    fi

    # Mark requirements as installed
    echo "$REQUIREMENTS_HASH" > "$MARKER_FILE"
    echo "Dependencies installed successfully."
else
    echo "Dependencies already up to date."
fi

# Run the GUI
echo "Starting MONOMOD GUI..."
cd "$SCRIPT_DIR"
python3 app.py "$@"
