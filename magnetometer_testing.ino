/*
#include <Arduino.h>
#include "Magnetometer.h"
#include <Wire.h>
#include "ArduinoLog.h"
const int MAGNETOMETER_DECLINATION_DEGS = 12;
const int MAGNETOMETER_DECLINATION_MINS = 52;
const char MAGNETOMETER_DECINATION_DIR = 'E';


Magnetometer magnetometer(MAGNETOMETER_DECLINATION_DEGS, MAGNETOMETER_DECLINATION_MINS);

void setup() {
  Serial.begin(9600);
  magnetometer.initialize();
  Log.begin(LOG_LEVEL_INFO, &Serial);
}

void loop() {
  float heading = magnetometer.GetHeadingDegrees();
  Log.infoln("Heading: %F", heading);

  Log.infoln("const double MAGNETOMETER_MIN_X = %F;", magnetometer.compass.minX);
  Log.infoln("const double MAGNETOMETER_MAX_X = %F;", magnetometer.compass.maxX);
  
  Log.infoln("const double MAGNETOMETER_MIN_Y = %F;", magnetometer.compass.minY);
  Log.infoln("const double MAGNETOMETER_MAX_Y = %F;", magnetometer.compass.maxY);

  Log.infoln("const double MAGNETOMETER_MIN_Z = %F;", magnetometer.compass.minZ);
  Log.infoln("const double MAGNETOMETER_MAX_Z = %F;", magnetometer.compass.maxZ);
}
*/