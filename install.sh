#!/usr/bin/env bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PACKAGE_NAME="$(basename "$SCRIPT_DIR")"
cd "$SCRIPT_DIR"

# Install Pip dependencies
# (If this package has no additional Python dependencies, you should delete requirements.txt)
cd "$SCRIPT_DIR"
if [ -f "./requirements.txt" ]; then
    pip install -r requirements.txt
fi
