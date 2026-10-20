#!/usr/bin/env bash

set -e

# ============================================================
# UVOT Firmware Upload
#
# Usage:
#   ./upload.sh <robot_type> <namespace>
#
# Examples:
#   ./upload.sh diff robot1
#   ./upload.sh skid robot2
#   ./upload.sh omni leader
# ============================================================

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <diff|skid|omni> <namespace>"
    exit 1
fi

ROBOT_TYPE="$1"
ROBOT_NAMESPACE="$2"

# ------------------------------------------------------------
# Select PlatformIO environment
# ------------------------------------------------------------
case "$ROBOT_TYPE" in
    diff)
        PIO_ENV="uvot_diff"
        ;;
    skid)
        PIO_ENV="uvot_skid"
        ;;
    omni)
        PIO_ENV="uvot_omni"
        ;;
    *)
        echo "Error: invalid robot type '$ROBOT_TYPE'"
        echo "Valid types: diff, skid, omni"
        exit 1
        ;;
esac

# ------------------------------------------------------------
# Validate ROS 2 namespace
# ------------------------------------------------------------
if [[ ! "$ROBOT_NAMESPACE" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]]; then
    echo "Error: invalid namespace '$ROBOT_NAMESPACE'"
    echo "Use letters, numbers and underscores."
    echo "The namespace must start with a letter."
    exit 1
fi

# ------------------------------------------------------------
# PlatformIO executable
# ------------------------------------------------------------
PIO="$HOME/.platformio/penv/bin/pio"

if [ ! -x "$PIO" ]; then
    echo "Error: PlatformIO not found at:"
    echo "  $PIO"
    exit 1
fi

echo
echo "========================================"
echo " UVOT firmware upload"
echo "========================================"
echo " Configuration : $ROBOT_TYPE"
echo " ROS namespace : $ROBOT_NAMESPACE"
echo " Environment   : $PIO_ENV"
echo "========================================"
echo

# Pass the namespace as a compile-time definition.
export PLATFORMIO_BUILD_FLAGS="-D ROBOT_NAMESPACE=\\\"${ROBOT_NAMESPACE}\\\""

"$PIO" run \
    -e "$PIO_ENV" \
    -t upload
