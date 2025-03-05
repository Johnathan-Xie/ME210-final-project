#include "IMU.h"


IMU::IMU()
{
  subEulerX = 0;
  subEulerY = 0;
  subEulerZ = 0;
}

void IMU::initialize() {
  bno = Adafruit_BNO055(55, 0x29);
  if (!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
  }
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);
  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      //displaySensorOffsets(calibrationData);

      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }
}
/** Get the heading of the compass in degrees. */
float IMU::GetHeadingDegrees()
{     
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> euler = bno.getQuat().toEuler();
    //double x = (euler.y() - subEulerY) * degToRad;
    //double y = (euler.z() - subEulerZ) * degToRad;
    //double z = (euler.x() - subEulerX) * degToRad;
    double z = euler.x() * degToRad;
    return z;
}