# specere-purger

under development, documentations are done later (or never)

what are we doing? attempting to communicate from a raspberry pi to:

 - Airflow Meter (I2C): [SFM4300-20-P](https://sensirion.com/media/documents/024CBAA1/62987ADF/Sensirion_Datasheet_SFM4300.pdf) 
 - Temp Sensor (I2C): [SHT45](https://cdn-shop.adafruit.com/product-files/6174/6174_Datasheet_SHT4x.pdf)
 - Oxygen Sensor (UART3): [LOX-02](https://sstsensing.com/product/luminox-oxygen-sensor/)
 - (TBD) Pressure sensor: Honeywell HSC series

what are we controlling? 2 valves: Purge flow valve and steady flow valve

what are our goals? make the o2 sensor reads 1% or less in the purged chamber
