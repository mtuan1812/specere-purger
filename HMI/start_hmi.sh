#!/usr/bin/env bash

# Navigate to the directory where the script is located
cd "$(dirname "$0")"

echo "Running pre-flight dependency checks..."

# 1. Check for gpiozero
if ! python3 -c "import gpiozero" 2>/dev/null; then
    echo "CRITICAL ERROR: Python module 'gpiozero' is not installed or cannot be imported."
    echo "System cannot safely control valves. Aborting startup."
    exit 1
fi

# 2. Check I2C Health
I2C_BUS=$(grep '^I2C_BUS' test.py | awk '{print $3}' | tr -d '\r')
if [ ! -c "/dev/i2c-${I2C_BUS}" ]; then
    echo "CRITICAL ERROR: I2C bus /dev/i2c-${I2C_BUS} is missing or not enabled."
    echo "Sensors will be completely unavailable. Aborting startup."
    exit 1
fi

# 3. Check UART Health
UART_PORT=$(grep '^UART_PORT' test.py | cut -d'"' -f2 | tr -d '\r')
if [ ! -c "$UART_PORT" ]; then
    echo "CRITICAL ERROR: UART port ${UART_PORT} is missing or not enabled."
    echo "LuminOx LOX sensor will be completely unavailable. Aborting startup."
    exit 1
fi

echo "All hardware dependencies verified. Proceeding..."

echo "Starting HMI Backend..."
# Run the Python server in the background and pipe output to hmi.log
python3 server.py > hmi.log 2>&1 &
echo "Backend started in background."

echo "Launching Chromium in Kiosk mode..."
# runuser -u admin -- env DISPLAY=:0 XAUTHORITY=/home/admin/.Xauthority \
#     chromium \
#     --kiosk \
#     --no-first-run \
#     --noerrdialogs \
#     --disable-infobars \
#     --disable-session-crashed-bubble \
#     --check-for-update-interval=31536000 \
#     http://localhost:8000 > /dev/null 2>&1 &
# echo "Chromium launched."

echo "Starting Swayidle Screen Manager..."
runuser -u admin -- env WAYLAND_DISPLAY=wayland-1 XDG_RUNTIME_DIR=/run/user/1000 ./screen_manager.sh &
echo "Screen manager launched."

echo "HMI initialization complete! System is running."

sleep 3
exit 0
