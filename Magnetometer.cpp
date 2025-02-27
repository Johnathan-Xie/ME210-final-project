#include <Arduino.h>
#include <Wire.h>
#include "Magnetometer.h"

Magnetometer::Magnetometer(int declination_degs, int declination_mins)
{
  this->declination_angle = (declination_degs + (declination_mins / 60.0)) / (180 / PI);
}

void Magnetometer::initialize() {
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
/** Get the heading of the compass in degrees. */
float Magnetometer::GetHeadingDegrees()
{     
  Vector norm = compass.readNormalize();
  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  heading += declination_angle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }
  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 
  return headingDegrees;
}