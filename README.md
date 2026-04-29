# specere-purger
Welcome to Specere Lab's OptiFlex chamber code repository!

what are we doing? attempting to communicate from a raspberry pi to:

 - Airflow Meter (I2C): [SFM4300-20-P](https://sensirion.com/media/documents/024CBAA1/62987ADF/Sensirion_Datasheet_SFM4300.pdf) 
 - Temp Sensor (I2C): [SHT45](https://cdn-shop.adafruit.com/product-files/6174/6174_Datasheet_SHT4x.pdf)
 - Oxygen Sensor (UART3): [LOX-02](https://sstsensing.com/product/luminox-oxygen-sensor/)
 - (TBD) Pressure sensor: Honeywell HSC series (no longer needed right now due to low pressure, but PCB footprint is still there)

what are we controlling? 2 valves: Purge flow valve and steady flow valve. 

what are our goals? make the o2 sensor reads 1% or less in the purged chamber. 

Refer to the report if you have any questions

## Setup

 - Plug empty microSD card into a PC 
 - Download latest version of
   [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
 - Flash the SD card with the following settings:
	 - Board: Raspberry Pi 4
	 - OS: Raspberry Pi OS (64 bit)
	- Hostname: set your desired computer name, default: `specere-eeoc`
	- Username: admin (please dont change this)
	- Password: refer to the report 
	- Wi-Fi: none (connect later)
	- Enable SSH with password authentication
	- Raspberry Pi Connect: unused, so whatever
- Flash the uSD card
- Done. 

Fire up the RPi. Connect it to Wi-Fi/Ethernet (using a phone as a personal hotspot recommended). Get to the console/terminal. There are 3 ways:
- via the touchscreen UI
- via SSH (find the IP address of the pi then via windows/mac/linux terminal) then `ssh -o StrictHostKeyChecking=no admin@<IP>`
- via the UART0 console port on the HAT PCB via a [USB to TTL adapter](https://www.amazon.com/HiLetgo-CP2102-Converter-Adapter-Downloader/dp/B00LODGRV8) with PuTTY or similar software. Remember to swap RX and TX

Once in, run this one line setup script:

    curl -fsSL https://raw.githubusercontent.com/mtuan1812/specere-purger/main/setup_rpi.sh | bash

Run the software with the desktop shortcut. Done!
