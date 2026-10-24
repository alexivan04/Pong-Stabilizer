#!/bin/bash
# deploy_teensy.sh for Pi5 + Teensy 4.1

set -euo pipefail

PI_USER="biban"
PI_IP="172.20.10.3"                     # adjust for your Pi5 hostname or IP
REMOTE_HEX="~/firmware.hex"

if [ -z "$1" ]; then
    echo "Usage: $0 path/to/firmware.hex"
    exit 1
fi

LOCAL_HEX="$1"

if [ ! -f "$LOCAL_HEX" ]; then
    echo "Error: firmware file not found: $LOCAL_HEX" >&2
    exit 1
fi

# ping check optional
echo "Checking if Pi5 is reachable..."
if ! ping -c 1 -W 3 "$PI_IP" > /dev/null 2>&1; then
    echo "Error: Pi5 not reachable"
    exit 1
fi

echo "Checking SSH on Pi5..."
if ! ssh -o ConnectTimeout=5 "$PI_USER@$PI_IP" true; then
    echo "Error: SSH is unavailable at $PI_USER@$PI_IP:22" >&2
    exit 1
fi

# copy firmware
echo "Copying firmware to Pi5..."
scp "$LOCAL_HEX" "$PI_USER@$PI_IP:$REMOTE_HEX"

# run flash script (pulse PROGRAM pin + flash Teensy)
echo "Flashing Teensy..."
ssh "$PI_USER@$PI_IP" "sudo python3 ~/main.py $REMOTE_HEX"
