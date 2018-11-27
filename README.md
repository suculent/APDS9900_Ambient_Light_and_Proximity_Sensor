APDS-9900 ALS Sensor Arduino Library
===============================================

This Arduino Library supports the APDS-9900 Ambient Light & Proximity Sensor sensor family. Forked from SparkFun APDS-9960.

Getting Started
---------------

* Download the Git repository as a ZIP ("Download ZIP" button)
* Unzip
* Copy the entire library directory (APDS-9900_ALS_Sensor_Arduino_Library
) to \<Arduino installation directory\>/libraries
* Open the Arduino program
* Select File -> Examples -> APDS9900 -> ProximitySensor
* Plug in your Arduino and APDS-9900 with the following connections

*-OR-*

* Use the library manager

| Arduino Pin | APDS-9900 Board | Function |
|---|---|---|
| 3.3V | VCC | Power |
| GND | GND | Ground |
| A4 | SDA | I2C Data |
| A5 | SCL | I2C Clock |
| 2 | INT | Interrupt |

* Go to Tools -> Board and select your Arduino board
* Go to Tools -> Serial Port and select the COM port of your Arduino board
* Click "Upload"
* Go to Tools -> Serial Monitor
* Ensure the baud rate is set at 9600 baud
* Swipe your hand over the sensor in various directions!

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/src** - Source files for the library (.cpp, .h).
* **library.properties** - General library properties for the Arduino package manager.


Version History
---------------

* V_1.0.0: Initial release

License Information
-------------------

This product is _**open source**_!

The **code** is beerware; if you see me at the local, and you've found our code helpful, please buy us a round!

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release anything derivative under the same license.

Distributed as-is; no warranty is given.

- Your friends at THiNX Cloud to friends at SparkFun.
