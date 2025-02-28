/*
#include <Arduino.h>
#include "Magnetometer.h"
#include <Wire.h>
const int MAGNETOMETER_DECLINATION_DEGS = 12;
const int MAGNETOMETER_DECLINATION_MINS = 52;
const char MAGNETOMETER_DECINATION_DIR = 'E';


Magnetometer magnetometer(MAGNETOMETER_DECLINATION_DEGS, MAGNETOMETER_DECLINATION_MINS);

void setup() {
  Serial.begin(9600);
  magnetometer.initialize();
}

void loop() {
  float heading = magnetometer.GetHeadingDegrees();
  Serial.println(heading);
  Serial.print("minX: ");
  Serial.println(magnetometer.compass.minX);
  
  Serial.print("maxX: ");;
  Serial.println(magnetometer.compass.maxX);
  
  Serial.print("minY: ");
  Serial.println(magnetometer.compass.minY);
  
  Serial.print("maxY: ");
  Serial.println(magnetometer.compass.maxY);
  
  Serial.print("minZ: ");
  Serial.println(magnetometer.compass.minZ);
  
  Serial.print("maxZ: ");
  Serial.println(magnetometer.compass.maxZ);
  
}
*/