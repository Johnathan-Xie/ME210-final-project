#include "DFRobot_QMC5883.h"

class Magnetometer
{
	public: 
        Magnetometer(int declination_degs, int declination_mins);

        // Get a heading in degrees
        float GetHeadingDegrees();
        void initialize();
        DFRobot_QMC5883 compass;
	private:
        float declination_angle = 0;  
};