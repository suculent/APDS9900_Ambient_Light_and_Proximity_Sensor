/****************************************************************
ProximitySensor.ino
APDS-9900 ALS and Proximity Sensor
Based on code by Shawn Hymel @ SparkFun Electronics
Maintained by Matej Sychra @ THiNX (github.com/suculent)
November 27, 2018
https://github.com/suculent/APDS9900_Ambient_Light_and_Proximity_Sensor

Tests the proximity sensing abilities of the APDS-9900.
Configures the APDS-9900 over I2C and polls for the distance to
the object nearest the sensor.

Hardware Connections:

IMPORTANT: The APDS-9900 can only accept 3.3V!
 
 Arduino Pin  APDS-9900 Board  Function
 
 3.3V         VCC              Power
 GND          GND              Ground
 D4           SDA              I2C Data
 D5           SCL              I2C Clock

Resources:
Include Wire.h and APDS-9900.h

Development environment specifics:
Written in Arduino 1.0.5-1.8.7
Tested with Wemos D1 Mini Pro

This code is beerware; if you see me (or any other SparkFun 
employee) at the local, and you've found our code helpful, please
buy us a round!

Distributed as-is; no warranty is given.
****************************************************************/

#include <Wire.h>
#include <APDS9900.h>

// Global Variables
APDS9900 apds = APDS9900();
uint8_t proximity_data = 0;

void setup() {

  Wire.begin(D1, D2); // A4, A5 is default for Arduino
  
  // Initialize Serial port
  Serial.begin(230400); // 9600 baud is default for Arduino
  Serial.println();
  Serial.println(F("------------------------------------"));
  Serial.println(F("    APDS-9900 - Proximity Sensor    "));
  Serial.println(F("------------------------------------"));
  
  // Initialize APDS-9900 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9900 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9900 init!"));
  }
  
  // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(0) ) {
    Serial.println(F("Something went wrong trying to set PGAIN"));
  }
  
  // Start running the APDS-9900 proximity sensor (no interrupts)
  if ( apds.enableProximitySensor(false) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
}

void loop() {
  
  // Read the proximity value
  if ( !apds.readProximity(proximity_data) ) {
    Serial.println("Error reading proximity value");
  } else {
    Serial.print("Proximity: ");
    Serial.println(proximity_data);
  }
  
  // Wait 250 ms before next reading
  delay(250);
}
