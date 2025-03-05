#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

class IMU
{
    public: 
      IMU();
      // Get a heading in degrees
      float GetHeadingDegrees();
      void initialize();
      double degToRad = 57.295779513;
      Adafruit_BNO055 bno;
	private:
    float subEulerX, subEulerY, subEulerZ;
};