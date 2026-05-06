#!/usr/bin/env bash
set -euo pipefail

DEFAULT_PX4_DIR="$HOME/PX4-Autopilot"
PX4_DIR="$DEFAULT_PX4_DIR"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -p|--px4-dir)
            PX4_DIR="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--px4-dir /path/to/PX4-Autopilot]"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--px4-dir /path/to/PX4-Autopilot]"
            exit 1
            ;;
    esac
done

if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4-Autopilot directory not found at: $PX4_DIR"
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
SRC_WORLD="$PACKAGE_DIR/cave_simple_03/simple_cave_03.sdf"
DST_WORLD="$PX4_DIR/Tools/simulation/gz/worlds/cave_simple_03.sdf"

if [ ! -f "$SRC_WORLD" ]; then
    echo "Error: source world file not found at: $SRC_WORLD"
    exit 1
fi

cp "$SRC_WORLD" "$DST_WORLD"
echo "Copied $SRC_WORLD -> $DST_WORLD"
