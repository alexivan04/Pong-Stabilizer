#!/bin/bash
# deploy_teensy.sh for Pi5 + Teensy 4.1

PI_USER="biban"
PI_IP="10.41.234.223"                     # adjust for your Pi5 hostname or IP
REMOTE_HEX="~/firmware.hex"

if [ -z "$1" ]; then
    echo "Usage: $0 path/to/firmware.hex"
    exit 1
fi

LOCAL_HEX="$1"

# ping check optional
echo "Checking if Pi5 is reachable..."
if ! ping -c 1 -W 3 "$PI_IP" > /dev/null 2>&1; then
    echo "Error: Pi5 not reachable"
    exit 1
fi

# copy firmware
echo "Copying firmware to Pi5..."
scp "$LOCAL_HEX" "$PI_USER@$PI_IP:$REMOTE_HEX"

# run flash script (pulse PROGRAM pin + flash Teensy)
echo "Flashing Teensy..."
ssh "$PI_USER@$PI_IP" "sudo python3 ~/main.py $REMOTE_HEX"
