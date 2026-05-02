#!/usr/bin/env bash

# Navigate to the directory where the script is located
cd "$(dirname "$0")"

# ==============================================================================
# Debounce / Single-instance Guard
# ==============================================================================
LOCK_FILE="/tmp/hmi.lock"
if [ -f "$LOCK_FILE" ]; then
    EXISTING_PID=$(cat "$LOCK_FILE")
    if kill -0 "$EXISTING_PID" 2>/dev/null; then
        echo "HMI is already running (PID $EXISTING_PID). Ignoring duplicate launch."
        exit 0
    else
        echo "Stale lock file found (PID $EXISTING_PID no longer alive). Removing..."
        rm -f "$LOCK_FILE"
    fi
fi
echo $$ > "$LOCK_FILE"
trap 'rm -f "$LOCK_FILE"' EXIT

# ==============================================================================
# Pre-flight Dependency Checks
# CRITICAL — abort on failure (core HMI / safety-critical)
# WARNING  — log and continue (degraded but non-fatal)
# ==============================================================================

WARNINGS=0

_critical() {
    echo ""
    echo "╔══════════════════════════════════════════╗"
    echo "║           CRITICAL ERROR — ABORT         ║"
    echo "╚══════════════════════════════════════════╝"
    echo "  $1"
    echo "  $2"
    echo ""
    exit 1
}

_warn() {
    echo "[WARN] $1"
    WARNINGS=$((WARNINGS + 1))
}

echo ""
echo "Running pre-flight dependency checks..."
echo "──────────────────────────────────────────────"

# --- 1. python3 binary ---
if ! command -v python3 &>/dev/null; then
    _critical "python3 is not installed or not in PATH." \
              "Run: sudo apt install python3"
fi
echo "[OK]   python3 found: $(python3 --version 2>&1)"

# --- 2. Critical Python modules (safety / core operation) ---
check_pymod_critical() {
    local mod="$1" pkg="$2"
    if ! python3 -c "import $mod" 2>/dev/null; then
        _critical "Python module '$mod' is missing." \
                  "Run: sudo apt install $pkg"
    fi
    echo "[OK]   python3 module: $mod"
}

check_pymod_critical "gpiozero"  "python3-gpiozero"
check_pymod_critical "smbus2"    "python3-smbus2"
check_pymod_critical "serial"    "python3-serial"
check_pymod_critical "PIL"       "python3-pil"
check_pymod_critical "qrcode"    "python3-qrcode"

# --- 3. Optional Python module (soft fallback in gpio_controller.py) ---
if ! python3 -c "import rpi_hardware_pwm" 2>/dev/null; then
    _warn "Python module 'rpi_hardware_pwm' not found. PWM will fall back to digital output (GPIO 19 always-on)."
    _warn "To fix: sudo pip3 install rpi-hardware-pwm --break-system-packages"
else
    echo "[OK]   python3 module: rpi_hardware_pwm"
fi

# --- 4. Required HMI asset files ---
check_file_critical() {
    local path="$1"
    if [ ! -f "$path" ]; then
        _critical "Required file missing: $path" \
                  "Re-clone the repository or check the installation."
    fi
    echo "[OK]   file: $path"
}

check_file_critical "server.py"
check_file_critical "web_server.py"
check_file_critical "app_state.py"
check_file_critical "gpio_controller.py"
check_file_critical "sensor_backend.py"
check_file_critical "test.py"
check_file_critical "index.html"

# --- 5. I2C bus device node ---
I2C_BUS=$(grep '^I2C_BUS' test.py | awk '{print $3}' | tr -d '\r')
if [ -z "$I2C_BUS" ]; then
    _critical "Could not parse I2C_BUS from test.py." \
              "Check that test.py defines I2C_BUS = <number>."
fi
if [ ! -c "/dev/i2c-${I2C_BUS}" ]; then
    _critical "I2C bus /dev/i2c-${I2C_BUS} is missing." \
              "Ensure 'dtparam=i2c_arm=on' is set in /boot/firmware/config.txt and reboot."
fi
echo "[OK]   I2C bus: /dev/i2c-${I2C_BUS}"

# --- 6. UART device node ---
UART_PORT=$(grep '^UART_PORT' test.py | cut -d'"' -f2 | tr -d '\r')
if [ -z "$UART_PORT" ]; then
    _critical "Could not parse UART_PORT from test.py." \
              "Check that test.py defines UART_PORT = \"<port>\"."
fi
if [ ! -c "$UART_PORT" ]; then
    _critical "UART port ${UART_PORT} is missing." \
              "Ensure 'dtoverlay=uart3' is set in /boot/firmware/config.txt and reboot."
fi
echo "[OK]   UART port: $UART_PORT"

# --- 7. chromium binary (required for kiosk HMI window) ---
if ! command -v chromium &>/dev/null; then
    _critical "chromium is not installed." \
              "Run: sudo apt install chromium"
fi
echo "[OK]   chromium found"

# --- 8. swayidle binary (screen dimming — non-fatal) ---
if ! command -v swayidle &>/dev/null; then
    _warn "swayidle not found. Screen auto-dimming will be unavailable."
    _warn "To fix: sudo apt install swayidle"
else
    echo "[OK]   swayidle found"
fi

# --- 9. evtest binary (touch-wake — non-fatal) ---
if ! command -v evtest &>/dev/null; then
    _warn "evtest not found. Touch-to-wake from dim will be unavailable."
    _warn "To fix: sudo apt install evtest"
else
    echo "[OK]   evtest found"
fi

# --- 10. Touch input device node (non-fatal) ---
TOUCH_DEV="/dev/input/event4"
if [ ! -e "$TOUCH_DEV" ]; then
    _warn "Touch device $TOUCH_DEV not found. screen_manager.sh may fail to wake on touch."
else
    echo "[OK]   touch device: $TOUCH_DEV"
fi

# --- 11. Backlight control node (non-fatal) ---
BL_PATH="/sys/class/backlight/10-0045/bl_power"
if [ ! -e "$BL_PATH" ]; then
    _warn "Backlight node $BL_PATH not found. Screen dimming will be unavailable."
else
    echo "[OK]   backlight node: $BL_PATH"
fi

echo "──────────────────────────────────────────────"
if [ "$WARNINGS" -gt 0 ]; then
    echo "Pre-flight complete with $WARNINGS warning(s). Starting in degraded mode..."
else
    echo "All dependency checks passed. Proceeding..."
fi
echo ""

# ==============================================================================
# Launch Services
# ==============================================================================

echo "Starting HMI Backend..."
python3 server.py > hmi.log 2>&1 &
echo "Backend started in background (PID $!)."

echo "Launching Chromium in Kiosk mode..."
runuser -u admin -- env DISPLAY=:0 XAUTHORITY=/home/admin/.Xauthority \
    chromium \
    --kiosk \
    --no-first-run \
    --noerrdialogs \
    --disable-infobars \
    --disable-session-crashed-bubble \
    --check-for-update-interval=31536000 \
    http://localhost:8000 > /dev/null 2>&1 &
echo "Chromium launched (PID $!)."

echo "Starting Swayidle Screen Manager..."
runuser -u admin -- env WAYLAND_DISPLAY=wayland-1 XDG_RUNTIME_DIR=/run/user/1000 ./screen_manager.sh &
echo "Screen manager launched (PID $!)."

echo ""
echo "HMI initialization complete! System is running."

sleep 3
exit 0
