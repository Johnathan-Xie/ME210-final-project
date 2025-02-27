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
}

void loop() {
  float heading = magnetometer.GetHeadingDegrees();
  Serial.println(heading);
  delay(200);
}
#include <Arduino.h>
#include "Magnetometer.h"
const int MAGNETOMETER_DECLINATION_DEGS = 12;
const int MAGNETOMETER_DECLINATION_MINS = 52;
const char MAGNETOMETER_DECINATION_DIR = 'E';


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
  float heading = magnetometer.GetHeadingDegrees();
  Serial.println(heading);
  delay(1000);
}



#include <Wire.h>
#include "DFRobot_QMC5883.h"
DFRobot_QMC5883 compass;
void setup()
{
  Serial.begin(9600);
  while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }
    if(compass.isHMC()){
        Serial.println("Initialize HMC5883");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
    }
   else if(compass.isQMC()){
        Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
   }
  }
void loop()
{
  Vector norm = compass.readNormalize();
  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (12.0 + (52.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }
  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 
  // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();
  delay(100);
}
*/