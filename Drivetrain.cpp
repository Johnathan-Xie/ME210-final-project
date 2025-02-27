#include <Arduino.h>

#include "Drivetrain.h"
#include "ArduinoLog.h"

void Drivetrain::initialize() {
    this->magnetometer.initialize();
}
void Drivetrain::set_target_location(float left_centimeters, float back_centimeters, float orientation_degrees) {
    this->target_left_centimeters = left_centimeters;
    this->target_back_centimeters = back_centimeters;
    this->target_orientation_degrees = orientation_degrees - this->reference_zero_orientation;
    Log.errorln("target_orientation_degrees: %F", target_orientation_degrees);
}
void Drivetrain::set_movement(float drive, float strafe, float twist, bool heading_correction = true) {
    this->update_measurements();
    if (heading_correction){
        float heading = get_last_measured_orientation_degrees() * (PI/180.0);
        float orig_drive = drive;
        float orig_strafe = strafe;
        drive = orig_drive * cos(heading) - orig_strafe * sin(heading);
        strafe = orig_drive * sin(heading) + orig_strafe * cos(heading);
    }
    
    Log.infoln("drive: %F", drive);
    Log.infoln("strafe: %F", strafe);
    Log.infoln("twist: %F", twist);

    float speeds[] = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
    };
    const int num_motors = sizeof(speeds) / sizeof(speeds[0]);

    // Because we are adding and motors only take values between
    // [-1,1] we may need to normalize them.
    //Find maximum value for speed normalization
    float max_speed = abs(speeds[0]);
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

float Drivetrain::get_left_distance_to_target_location() {
    return this->target_left_centimeters - this->last_measured_left_centimeters;
}

float Drivetrain::get_back_distance_to_target_location() {
    return this->target_back_centimeters - this->last_measured_back_centimeters;
}

float Drivetrain::get_degrees_to_target_orientation() {
    return this->target_orientation_degrees - this->last_measured_orientation_degrees;
}
/*
float Drivetrain::get_distance_to_target_location() {
    return sqrt(
        sq(this->get_left_distance_to_target_location()) +
        sq(this->get_back_distance_to_target_location())
    );
}
*/

void Drivetrain::update_measurements() {
    float new_left_centimeters = (float)this->left_ultrasonic.read();
    float new_back_centimeters = (float)this->back_ultrasonic.read();
    float new_orientation_degrees = magnetometer.GetHeadingDegrees() - this->reference_zero_orientation;
    Log.errorln("new_orientation_degrees: %F", new_orientation_degrees);
    Log.infoln("new_left_centimeters: %F", new_left_centimeters);
    Log.infoln("new_back_centimeters: %F", new_back_centimeters);
    Log.infoln("new_orientaton_degrees: %F", new_orientation_degrees);
    
    this->last_measured_left_centimeters = new_left_centimeters;
    if ((abs(new_left_centimeters - this->last_measured_left_centimeters) <= this->max_allowed_left_centimeters_change) || this->last_measured_back_centimeters < -999) {
    }
    this->last_measured_back_centimeters = new_back_centimeters;
    if ((abs(new_back_centimeters - this->last_measured_back_centimeters) <= this->max_allowed_back_centimeters_change) || this->last_measured_back_centimeters < -999) {
    }
    this->last_measured_orientation_degrees = new_orientation_degrees;
    if ((abs(new_orientation_degrees - this->last_measured_orientation_degrees) <= this->max_allowed_orientation_degrees_change) || this->last_measured_orientation_degrees < -999) {
        
    }
    Log.infoln("updated left centimeters %F", this->last_measured_left_centimeters);
    Log.infoln("updated back centimeters %F", this->last_measured_back_centimeters);
}

bool Drivetrain::update_towards_target_location(
    bool update_left = true,
    bool update_back = true,
    bool update_orientation = true,
    float back_centimeters_tolerance = -1.0,
    float left_centimeters_tolerance = -1.0,
    float orientation_degrees_tolerance = -1.0
) {
    this->update_measurements();
    float left_distance_to_target = this->get_left_distance_to_target_location();
    float back_distance_to_target = this->get_back_distance_to_target_location();
    float degrees_to_target_orientation = this->get_degrees_to_target_orientation();
    
    Log.errorln("degrees_to_target_orientation: %F", degrees_to_target_orientation);
    if (!update_left|| abs(left_distance_to_target) < this->stop_left_centimeters) {
        left_distance_to_target = 0;
    }
    if (!update_back || abs(back_distance_to_target) < this->stop_back_centimeters) {
        back_distance_to_target = 0;
    }
    Log.errorln("(abs(degrees_to_target_orientation) < this->stop_degrees): %d", (abs(degrees_to_target_orientation) < this->stop_degrees));
    Log.errorln("(abs(degrees_to_target_orientation) > 360 - this->stop_degrees): %d", (abs(degrees_to_target_orientation) > 360 - this->stop_degrees));
    if (!update_orientation || (abs(degrees_to_target_orientation) < this->stop_degrees) || (abs(degrees_to_target_orientation) > 360 - this->stop_degrees)) {
        degrees_to_target_orientation = 0;
    }
    Log.errorln("degrees_to_target_orientation: %F", degrees_to_target_orientation);
    float max_distance = max(abs(back_distance_to_target), abs(left_distance_to_target));
    float drive = back_distance_to_target;
    float strafe = left_distance_to_target;
    if (max_distance > 0){
        float drive = drive / max_distance;
        float strafe = strafe / max_distance;
    }
    float twist;
    if (degrees_to_target_orientation < -180.0) {
        twist = (360.0 - degrees_to_target_orientation) / this->twist_divisor;
    } else{
        twist = degrees_to_target_orientation / this->twist_divisor;
    }
    if (twist < 0) {
        twist = max(twist, -this->max_twist);
    } else {
        twist = min(twist, this->max_twist);
    }
    Log.errorln("twist: %F", twist);
    if (abs(left_distance_to_target) < this->begin_linear_slowdown_left_centimeters) {
        drive = drive * (left_distance_to_target - this->stop_left_centimeters)
                        / (this->begin_linear_slowdown_left_centimeters - this->stop_left_centimeters);
    }
    if (abs(back_distance_to_target) < this->begin_linear_slowdown_back_centimeters) {
        strafe = strafe * (back_distance_to_target - this->stop_back_centimeters)
                        / (this->begin_linear_slowdown_back_centimeters - this->stop_back_centimeters);
    }
    if (abs(degrees_to_target_orientation) < this->begin_linear_slowdown_degrees) {
        twist = twist * (degrees_to_target_orientation - this->stop_degrees)
                        / (this->begin_linear_slowdown_degrees - this->stop_degrees);
    }
    
    Log.errorln("twist: %F", twist);
    this->set_movement(drive, strafe, twist);
    if (left_centimeters_tolerance < 0) {
        left_centimeters_tolerance = this->stop_left_centimeters;
    }
    if (back_centimeters_tolerance < 0) {
        back_centimeters_tolerance = this->stop_back_centimeters;
    }
    if (orientation_degrees_tolerance < 0) {
        orientation_degrees_tolerance = this->stop_degrees;
    }
    bool all_within_tolerance = (
        (left_distance_to_target < left_centimeters_tolerance) &&
        (back_distance_to_target < back_centimeters_tolerance) &&
        (degrees_to_target_orientation < orientation_degrees_tolerance)
    );
    Log.infoln("All within tolerance: %d", all_within_tolerance);
    return all_within_tolerance;
}

float Drivetrain::get_last_measured_left_centimeters() {
    return this->last_measured_left_centimeters;
}
float Drivetrain::get_last_measured_back_centimeters() {
    return this->last_measured_back_centimeters;
}
float Drivetrain::get_last_measured_orientation_degrees(){
    return this->last_measured_orientation_degrees;
}
