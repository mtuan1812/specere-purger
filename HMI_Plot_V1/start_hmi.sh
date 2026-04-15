#!/usr/bin/env bash

# Navigate to the directory where the script is located
cd "$(dirname "$0")"

echo "Starting HMI Backend..."
# Run the Python server in the background and pipe output to hmi.log
python3 server.py > hmi.log 2>&1 &
echo "Backend started in background."

echo "Launching Chromium in Kiosk mode..."
runuser -u admin -- env DISPLAY=:0 XAUTHORITY=/home/admin/.Xauthority \
    chromium \
    --kiosk \
    --no-first-run \
    --noerrdialogs \
    --disable-infobars \
    --disable-session-crashed-bubble \
    --check-for-update-interval=31536000 \
    http://localhost:8000 &
echo "Chromium launched."

echo "HMI initialization complete! System is running."
