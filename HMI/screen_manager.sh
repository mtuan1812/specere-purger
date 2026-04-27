#!/bin/bash

DEVICE="/dev/input/event4"
BL="/sys/class/backlight/rpi_backlight/bl_power"

if [ "$1" == "dim" ]; then
    # Turn off backlight
    # Note: Using sudo tee to ensure we have permissions
    echo 1 | sudo tee "$BL" > /dev/null
    
    # Grab the touch device so Wayland doesn't see the touch.
    # evtest will output events. We grep for touch down.
    sudo evtest --grab "$DEVICE" | while read -r line; do
        if echo "$line" | grep -q "BTN_TOUCH.*value 1"; then
            # Turn backlight back on
            echo 0 | sudo tee "$BL" > /dev/null
            
            # Kill evtest to release the grab
            sudo killall evtest
            break
        fi
    done
    exit 0
fi

SCRIPT_PATH="$(realpath "$0")"

export WAYLAND_DISPLAY="wayland-1"
export XDG_RUNTIME_DIR="/run/user/1000"

killall swayidle 2>/dev/null

echo "Starting swayidle..."
exec swayidle -w \
    timeout 300 "$SCRIPT_PATH dim" \
    resume "echo 0 | sudo tee $BL > /dev/null"
