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
}
*/