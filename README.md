# Specere Purger (OptiFlex Chamber)

Control and monitoring system for the Specere Lab OptiFlex chamber, running on Raspberry Pi.

## Project Overview
The system monitors environmental conditions and controls gas flow to achieve and maintain a purged state (≤1% $O_2$) within the chamber.

### Core Objectives
- Achieve $O_2$ levels of 1% or less in the purged chamber.
- Automate dual-valve control (Purge flow and Steady flow).
- Provide real-time telemetry via a local HMI.

## Hardware Specifications

### Sensors
- **Airflow Meter (I2C):** [Sensirion SFM4300-20-P](https://sensirion.com/media/documents/024CBAA1/62987ADF/Sensirion_Datasheet_SFM4300.pdf)
- **Temp/Humidity (I2C):** [Sensirion SHT45](https://cdn-shop.adafruit.com/product-files/6174/6174_Datasheet_SHT4x.pdf)
- **Oxygen Sensor (UART3):** [SST Sensing LOX-02](https://sstsensing.com/product/luminox-oxygen-sensor/)
- **Pressure (Optional):** Honeywell HSC series (Footprint available, currently unused due to low pressure).

### Actuators
- **Purge Flow Valve:** GPIO-controlled solenoid.
- **Steady Flow Valve:** GPIO-controlled solenoid.

---

## Installation & Setup

### 1. OS Preparation
1. Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to flash a microSD card.
2. **Settings:**
   - **Board:** Raspberry Pi 4
   - **OS:** Raspberry Pi OS (64-bit)
   - **Hostname:** `specere-eeoc` (Recommended default)
   - **Username:** `admin` (Required for automated scripts)
   - **Password:** [Refer to Project Report]
   - **Wi-Fi:** None (Configure after first boot)
3. **Services:** 
   - Enable **SSH** with password authentication.
   - **Raspberry Pi Connect:** Optional/Unused.

### 2. Software Installation
Once booted and connected to the internet, run the automated setup script:
```bash
curl -fsSL https://raw.githubusercontent.com/mtuan1812/specere-purger/main/setup_rpi.sh | bash
```

## Operation

### Running the Application
- **HMI:** Use the desktop shortcut created by the setup script.
- **Auto-Execute:** If prompted to "Execute File", click **Execute**. 
  - *Tip: To skip this prompt in the future, go to File Explorer > Edit > Preferences > General and check "Don't ask options on launch executable files".*

### Troubleshooting & Console Access
Access the RPi terminal via:
1. **Direct Touchscreen:** Open the terminal emulator.
2. **SSH:** `ssh -o StrictHostKeyChecking=no admin@<IP_ADDRESS>`
3. **Serial Console:** Use the UART0 port on the HAT PCB via a USB-to-TTL adapter (115200 baud). Remember to swap RX/TX.

---
*For detailed engineering logic and safety protocols, refer to the full Project Report.*