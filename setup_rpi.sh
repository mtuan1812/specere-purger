#!/bin/bash
set -e

USER_NAME="admin"
HOME_DIR="/home/$USER_NAME"
GIT_DIR="$HOME_DIR/Git"
REPO_URL="https://github.com/mtuan1812/specere-purger.git"
REPO_DIR="$GIT_DIR/specere-purger"
CONFIG_FILE="/boot/firmware/config.txt"

echo "Updating system..."
sudo apt update
sudo apt upgrade -y

echo "Installing apt packages..."
# Installing core dependencies, gpiozero, and utilities for screen dimming (swayidle, evtest)
sudo apt install -y \
  git \
  build-essential \
  python3-gpiozero \
  python3-rpi-lgpio \
  python3-serial \
  python3-smbus2 \
  python3-pil \
  python3-qrcode \
  evtest \
  swayidle \
  chromium-browser \
  python3-pip

echo "Installing pip packages..."
sudo pip3 install rpi-hardware-pwm --break-system-packages

echo "Creating Git directory..."
mkdir -p "$GIT_DIR"
sudo chown -R "$USER_NAME:$USER_NAME" "$GIT_DIR"

if [ -d "$REPO_DIR/.git" ]; then
    echo "Repo already exists. Pulling latest..."
    cd "$REPO_DIR"
    git pull
else
    echo "Cloning repo..."
    git clone "$REPO_URL" "$REPO_DIR"
fi

sudo chown -R "$USER_NAME:$USER_NAME" "$REPO_DIR"

echo "Updating Raspberry Pi boot config..."
if [ -f "$CONFIG_FILE" ]; then
    sudo cp "$CONFIG_FILE" "$CONFIG_FILE.backup.$(date +%Y%m%d_%H%M%S)"
    sudo sed -i '/# SPECERE PURGER CONFIG START/,/# SPECERE PURGER CONFIG END/d' "$CONFIG_FILE"
else
    echo "Warning: $CONFIG_FILE not found, creating a new one..."
fi

sudo tee -a "$CONFIG_FILE" > /dev/null <<'EOF'

# SPECERE PURGER CONFIG START
dtparam=i2c_arm=on,i2c_arm_baudrate=100000
dtoverlay=uart3
dtoverlay=uart5
enable_uart=1
dtoverlay=dwc2,dr_mode=peripheral
dtoverlay=gpio-fan,gpiopin=18,temp=60000
dtoverlay=pwm-2chan
# SPECERE PURGER CONFIG END
EOF

echo "Creating desktop shortcut..."
mkdir -p "$HOME_DIR/Desktop"
DESKTOP_FILE="$HOME_DIR/Desktop/startup.desktop"

cat > "$DESKTOP_FILE" <<'EOF'
[Desktop Entry]
Name=HMI Start
Exec=lxterminal -e bash -c 'sudo /home/admin/Git/specere-purger/HMI/start_hmi.sh; echo; read -p "Press Enter to close..."'
Icon=utilities-system-monitor
Terminal=false
Type=Application
EOF

chmod +x "$DESKTOP_FILE"
sudo chown "$USER_NAME:$USER_NAME" "$DESKTOP_FILE"

echo "Setup complete!"
echo "Please restart the Raspberry Pi to apply the new boot configuration."
echo "Run: sudo reboot"
