/****************************************************************
AmbientLightInterrupt.ino
APDS-9900 RGB and Gesture Sensor
Shawn Hymel @ SparkFun Electronics
October 24, 2014
https://github.com/sparkfun/APDS-9900_RGB_and_Gesture_Sensor

Tests the ambient light interrupt abilities of the APDS-9900.
Configures the APDS-9900 over I2C and waits for an external
interrupt based on high or low light conditions. Try covering
the sensor with your hand or bringing the sensor close to a
bright light source. You might need to adjust the LIGHT_INT_HIGH
and LIGHT_INT_LOW values to get the interrupt to work correctly.

Hardware Connections:

IMPORTANT: The APDS-9900 can only accept 3.3V!
 
 Arduino Pin  APDS-9900 Board  Function
 
 3.3V         VCC              Power
 GND          GND              Ground
 A4           SDA              I2C Data
 A5           SCL              I2C Clock
 2            INT              Interrupt
 13           -                LED

Resources:
Include Wire.h and APDS-9900.h

Development environment specifics:
Written in Arduino 1.0.5
Tested with SparkFun Arduino Pro Mini 3.3V

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

// Constants
#define LIGHT_INT_HIGH  1000 // High light level for interrupt
#define LIGHT_INT_LOW   10   // Low light level for interrupt

// Global variables
APDS9900 apds = APDS9900();
uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
int isr_flag = 0;
uint16_t threshold = 0;

void interruptRoutine() {
  isr_flag = 1;
}

void setup() {
  
  // Set LED as output
  pinMode(LED_PIN, OUTPUT);
  pinMode(APDS9900_INT, INPUT);
  
  // Initialize Serial port
  Serial.begin(230400);
  Serial.println();
  Serial.println(F("-------------------------------------"));
  Serial.println(F("SparkFun APDS-9900 - Light Interrupts"));
  Serial.println(F("-------------------------------------"));
  
  // Initialize interrupt service routine
  attachInterrupt(0, interruptRoutine, FALLING);
  
  // Initialize APDS-9900 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9900 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9900 init!"));
  }
  
  // Set high and low interrupt thresholds
  if ( !apds.setLightIntLowThreshold(LIGHT_INT_LOW) ) {
    Serial.println(F("Error writing low threshold"));
  }
  if ( !apds.setLightIntHighThreshold(LIGHT_INT_HIGH) ) {
    Serial.println(F("Error writing high threshold"));
  }
  
  // Start running the APDS-9900 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }
  
  // Read high and low interrupt thresholds
  if ( !apds.getLightIntLowThreshold(threshold) ) {
    Serial.println(F("Error reading low threshold"));
  } else {
    Serial.print(F("Low Threshold: "));
    Serial.println(threshold);
  }
  if ( !apds.getLightIntHighThreshold(threshold) ) {
    Serial.println(F("Error reading high threshold"));
  } else {
    Serial.print(F("High Threshold: "));
    Serial.println(threshold);
  }
  
  // Enable interrupts
  if ( !apds.setAmbientLightIntEnable(true) ) {
    Serial.println(F("Error enabling interrupts"));
  }
  
  // Wait for initialization and calibration to finish
  delay(500);
}

void loop() {

    // If interrupt occurs, print out the light levels, else exit.
  if ( isr_flag == 0 ) return;
    
  // Read the light levels (ambient, red, green, blue) and print
  if (  !apds.readAmbientLight(ambient_light) ) {          
    Serial.println("Error reading light values");
  } else {
    Serial.print("Interrupt! Ambient: ");
    Serial.print(ambient_light);   
  }
  
  // Turn on LED for a half a second
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  // Reset flag and clear APDS-9900 interrupt (IMPORTANT!)
  isr_flag = 0;
  if ( !apds.clearAmbientLightInt() ) {
    Serial.println("Error clearing interrupt");
  }
}
