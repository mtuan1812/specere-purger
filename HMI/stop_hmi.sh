#!/usr/bin/env bash

echo "Stopping Chromium processes..."
killall chromium 2>/dev/null || echo "Chromium was not running."
killall chromium-browser 2>/dev/null || true

echo "Stopping HMI Python Backend..."
# SIGTERM first to allow graceful shutdown (flushes CSV buffer, closes GPIO)
pkill -SIGTERM -f "python3 server.py" 2>/dev/null || echo "server.py was not running."
# Give it up to 5 seconds to exit cleanly before force-killing
for i in $(seq 1 5); do
    pgrep -f "python3 server.py" > /dev/null 2>&1 || break
    sleep 1
done
# Force kill if still alive
if pgrep -f "python3 server.py" > /dev/null 2>&1; then
    echo "Backend did not exit cleanly. Sending SIGKILL..."
    pkill -SIGKILL -f "python3 server.py" 2>/dev/null || true
fi

echo "Stopping Screen Manager..."
pkill -f "screen_manager.sh" 2>/dev/null || true
killall swayidle 2>/dev/null || true
sudo killall evtest 2>/dev/null || true

echo "Cleaning up lock file..."
rm -f /tmp/hmi.lock

echo "HMI successfully stopped."
