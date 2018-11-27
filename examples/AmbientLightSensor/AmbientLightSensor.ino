/****************************************************************
AmbientLightSensor.ino
APDS-9900 ALS and Proximity Sensor
Based on code by Shawn Hymel @ SparkFun Electronics
Maintained by Matej Sychra @ THiNX (github.com/suculent)
November 27, 2018
https://github.com/suculent/APDS9900_Ambient_Light_and_Proximity_Sensor

Tests the ambient light abilities of the APDS-9900.
Configures the APDS-9900 over I2C and polls the values.

Hardware Connections:

IMPORTANT: The APDS-9900 can only accept 3.3V!
 
 Arduino Pin  APDS-9900 Board  Function
 
 3.3V         VCC              Power
 GND          GND              Ground
 D1           SDA              I2C Data
 D2           SCL              I2C Clock
 D4           -                LED

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

// Pins
#define APDS9900_INT    D8  // Needs to be an interrupt pin
#define LED_PIN         D4 // LED for showing interrupt

// Global variables
APDS9900 apds = APDS9900();
uint16_t ambient_light = 0;

void setup() {

  Wire.begin(D1, D2); // A4, A5 is default for Arduino
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(APDS9900_INT, INPUT);
  
  // Initialize Serial port
  Serial.begin(230400); // 9600 baud is default for Arduino
  Serial.println();
  Serial.println(F("-------------------------------------"));
  Serial.println(F("  APDS-9900 - Ambient Light Sensor   "));
  Serial.println(F("-------------------------------------"));
  
  // Initialize APDS-9900 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9900 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9900 init!"));
  }
  
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }
  
  // Wait for initialization and calibration to finish
  delay(500);
}

void loop() {    
  apds.readAmbientLight(ambient_light);
  Serial.print("Ambient light: ");
  Serial.println(ambient_light);         
  delay(250);
}
