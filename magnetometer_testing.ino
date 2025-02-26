/*
#include <Arduino.h>
#include "Magnetometer.h"
const int MAGNETOMETER_DECLINATION_DEGS = 12;
const int MAGNETOMETER_DECLINATION_MINS = 52;
const char MAGNETOMETER_DECINATION_DIR = 'E'


Magnetometer magnetometer(MAGNETOMETER_DECLINATION_DEGS, MAGNETOMETER_DECLINATION_MINS, MAGNETOMETER_DECINATION_DIR);

void setup() {
  Serial.begin(9600);
  magnetometer.SetDeclination(
    MAGNETOMETER_DECLINATION_DEGS,
    MAGNETOMETER_DECLINATION_MINS,
    MAGNETOMETER_DECINATION_DIR
  );
}

void loop() {
  float heading = Compass.GetHeadingDegrees();
  Serial.println(heading);
  delay(200);
}
*/