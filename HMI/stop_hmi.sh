#!/usr/bin/env bash

echo "Stopping Chromium processes..."
killall chromium 2>/dev/null || echo "Chromium was not running."
killall chromium-browser 2>/dev/null || true

echo "Stopping HMI Python Backend..."
# Uses pkill to specifically target the python backend running server.py
pkill -f "python3 server.py" 2>/dev/null || echo "server.py was not running."

echo "Stopping Screen Manager..."
pkill -f "screen_manager.sh" 2>/dev/null || true
killall swayidle 2>/dev/null || true
sudo killall evtest 2>/dev/null || true

echo "HMI successfully stopped."
