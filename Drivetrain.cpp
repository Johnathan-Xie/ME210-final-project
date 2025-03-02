#include <Arduino.h>

#include "Drivetrain.h"
#include "ArduinoLog.h"

void Drivetrain::initialize(
    double magnetometer_min_x = 0,
    double magnetometer_max_x = 0,
    double magnetometer_min_y = 0,
    double magnetometer_max_y = 0,
    double magnetometer_min_z = 0,
    double magnetometer_max_z = 0
)
{
    this->magnetometer.initialize();
    this->magnetometer.compass.setMinMax(
        magnetometer_min_x,
        magnetometer_max_x,
        magnetometer_min_y,
        magnetometer_max_y,
        magnetometer_min_z,
        magnetometer_max_z
    );
}

void Drivetrain::set_target_location(double left_centimeters, double back_centimeters, double orientation_degrees) {
    this->target_left_centimeters = left_centimeters;
    this->target_back_centimeters = back_centimeters;
    this->target_orientation_degrees = orientation_degrees;
}
void Drivetrain::set_movement(double drive, double strafe, double twist, bool heading_correction = true) {
    this->update_measurements();
    if (heading_correction){
        double heading = radians(get_last_measured_orientation_degrees());
        double orig_drive = drive;
        double orig_strafe = strafe;
        drive = orig_drive * cos(heading) - orig_strafe * sin(heading);
        strafe = orig_drive * sin(heading) + orig_strafe * cos(heading);
    }
    /*
    Log.infoln("drive: %F", drive);
    Log.infoln("strafe: %F", strafe);
    Log.infoln("twist: %F", twist);   
    */
    

    double speeds[] = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
    };
    const int num_motors = sizeof(speeds) / sizeof(speeds[0]);

    // Because we are adding and motors only take values between
    // [-1,1] we may need to normalize them.
    //Find maximum value for speed normalization
    double max_speed = abs(speeds[0]);
    for(int i = 1; i < num_motors; i++) {
        max_speed = max(max_speed, abs(speeds[i]));
    }

    // If and only if the maximum is outside of the range we want it to be,
    // normalize all the other speeds based on the given speed value.
    if (max_speed > 1) {
      for (int i = 0; i < num_motors; i++) {
            speeds[i] /= max_speed;
        }
    }
    
    Log.infoln("front_left speed: %F", speeds[0]);
    Log.infoln("front_right speed: %F", speeds[1]);
    Log.infoln("back_left speed: %F", speeds[2]);
    Log.infoln("back_right speed: %F", speeds[3]);
    this->front_left_motor.set_speed(speeds[0]);
    this->front_right_motor.set_speed(speeds[1]);
    this->back_left_motor.set_speed(speeds[2]);
    this->back_right_motor.set_speed(speeds[3]);
}

double Drivetrain::get_left_distance_to_target_location() {
    return this->target_left_centimeters - this->last_measured_left_centimeters;
}

double Drivetrain::get_back_distance_to_target_location() {
    return this->target_back_centimeters - this->last_measured_back_centimeters;
}

double Drivetrain::get_degrees_to_target_orientation() {
    return degrees_atan2(this->target_orientation_degrees, this->last_measured_orientation_degrees);
}
/*
double Drivetrain::get_distance_to_target_location() {
    return sqrt(
        sq(this->get_left_distance_to_target_location()) +
        sq(this->get_back_distance_to_target_location())
    );
}
*/
double Drivetrain::degrees_atan2(double a, double b) {
    // Computes the difference between angles a and b (in degrees)
    double diff_rad = atan2(sin(radians(a - b)), cos(radians(a - b)));
    return degrees(diff_rad);
}

void Drivetrain::update_measurements(
    bool update_left = true,
    bool update_back = true,
    bool update_orientation = true
) {
    if (update_back) {
        double new_back_centimeters = (double)this->back_ultrasonic.read();
        this->last_measured_back_centimeters = new_back_centimeters;
        if ((abs(new_back_centimeters - this->last_measured_back_centimeters) <= this->max_allowed_back_centimeters_change) || this->last_measured_back_centimeters < -999) {
            
        }
        Log.infoln("updated back centimeters %F", this->last_measured_back_centimeters);
    }
    if (update_left) {
        double new_left_centimeters = (double)this->left_ultrasonic.read();
        this->last_measured_left_centimeters = new_left_centimeters;
        if ((abs(new_left_centimeters - this->last_measured_left_centimeters) <= this->max_allowed_left_centimeters_change) || this->last_measured_back_centimeters < -999) {
            
        }
        Log.infoln("updated left centimeters %F", this->last_measured_left_centimeters);
    }
    
    if (update_orientation) {
        double new_orientation_degrees = degrees_atan2((double)magnetometer.GetHeadingDegrees(), this->reference_zero_orientation);
        this->last_measured_orientation_degrees = new_orientation_degrees;
        //if ((abs(degrees_atan2(new_orientation_degrees, this->last_measured_orientation_degrees)) <= this->max_allowed_orientation_degrees_change) || this->last_measured_orientation_degrees < -999) {   
        //}
        Log.infoln("updated last_measured_orientation_degrees %F", this->last_measured_orientation_degrees);
    }
}

double Drivetrain::clip_max(
    double value,
    double max_value
) {
    if (value < 0) {
        return max(value, -max_value);
    } else if (value < 0.01) {
        return 0;
    } else {
        return min(value, max_value);
    }
}

bool Drivetrain::update_towards_target_location(
    bool update_left = true,
    bool update_back = true,
    bool update_orientation = true,
    bool heading_correction = true,
    double back_centimeters_tolerance = -1.0,
    double left_centimeters_tolerance = -1.0,
    double orientation_degrees_tolerance = -1.0
) {
    this->update_measurements();
    double left_distance_to_target = this->get_left_distance_to_target_location();
    double back_distance_to_target = this->get_back_distance_to_target_location();
    double degrees_to_target_orientation = this->get_degrees_to_target_orientation();
    if (!update_left|| abs(left_distance_to_target) < this->stop_left_centimeters) {
        left_distance_to_target = 0;
    }
    if (!update_back || abs(back_distance_to_target) < this->stop_back_centimeters) {
        back_distance_to_target = 0;
    }
    if (!update_orientation || (abs(degrees_to_target_orientation) < this->stop_degrees)) {
        degrees_to_target_orientation = 0;
    }
    double max_distance = max(abs(back_distance_to_target), abs(left_distance_to_target));
    double drive = back_distance_to_target;
    double strafe = left_distance_to_target; 
    if (max_distance > 0) {
        drive = drive / max_distance;
        strafe = strafe / max_distance;
    }
    drive = this->clip_max(drive, this->max_drive);
    strafe = this->clip_max(strafe, this->max_strafe);

    double twist = degrees_to_target_orientation / twist_divisor;
    twist = this->clip_max(twist, this->max_twist);
    
    
    /*
    if (abs(left_distance_to_target) < this->begin_linear_slowdown_left_centimeters) {
        drive = drive * (abs(left_distance_to_target) - this->stop_left_centimeters)
                        / (this->begin_linear_slowdown_left_centimeters - this->stop_left_centimeters);
    }
    if (abs(back_distance_to_target) < this->begin_linear_slowdown_back_centimeters) {
        strafe = strafe * (abs(back_distance_to_target) - this->stop_back_centimeters)
                        / (this->begin_linear_slowdown_back_centimeters - this->stop_back_centimeters);
    }
    */
    if (abs(degrees_to_target_orientation) < this->begin_linear_slowdown_degrees || 180 - abs(degrees_to_target_orientation) < this->begin_linear_slowdown_degrees) {
        twist = twist * abs((abs(degrees_to_target_orientation) - this->stop_degrees)
                        / (this->begin_linear_slowdown_degrees - this->stop_degrees));
    }
    
    Log.infoln("drive: %F", drive);
    Log.infoln("strafe: %F", strafe);
    Log.infoln("twist: %F", twist);
    this->set_movement(drive, strafe, twist, heading_correction);
    if (left_centimeters_tolerance < 0) {
        left_centimeters_tolerance = this->stop_left_centimeters;
    }
    if (back_centimeters_tolerance < 0) {
        back_centimeters_tolerance = this->stop_back_centimeters;
    }
    if (orientation_degrees_tolerance < 0) {
        orientation_degrees_tolerance = this->stop_degrees;
    }
    Log.infoln("left_distance_to_target: %F", left_distance_to_target);
    Log.infoln("back_distance_to_target: %F", back_distance_to_target);
    Log.infoln("degrees_to_target_orientation: %F", degrees_to_target_orientation);
    bool all_within_tolerance = (
        (abs(left_distance_to_target) < left_centimeters_tolerance) &&
        (abs(back_distance_to_target) < back_centimeters_tolerance) &&
        (abs(degrees_to_target_orientation) < orientation_degrees_tolerance)
    );
    Log.infoln("All within tolerance: %d", all_within_tolerance);
    return all_within_tolerance;
}

double Drivetrain::get_last_measured_left_centimeters() {
    return this->last_measured_left_centimeters;
}
double Drivetrain::get_last_measured_back_centimeters() {
    return this->last_measured_back_centimeters;
}
double Drivetrain::get_last_measured_orientation_degrees(){
    return this->last_measured_orientation_degrees;
}
