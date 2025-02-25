#include <Arduino.h>
#include "Motor.h"
#include "Ultrasonic.h"
#include "Magnetometer.h"

#include "Drivetrain.h"

void set_target_location(float left_centimeters, float back_centimeters, float orientation_degrees) {
    this->target_left_centimeters = left_centimeters;
    this->target_back_centimeters = back_centimeters;
    this->target_orientation_degrees = orientation_degrees - this->reference_zero_orientation;
}
void set_movement(float drive, float strafe, float twist, bool heading_correction = true) {
    if (heading_correction){
        float heading = get_last_measured_orientation_degrees() * (PI/180.0);
        drive = strafe * cos(heading) - drive * sin(heading);
        strafe = strafe * sin(heading) + drive * cos(heading);
    }

    float[] speeds = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
    };

    //apply signs in case values need reversal
    int[] signs = {1, -1, 1, -1};
    for(int i=0; i<speeds.length; i++)
        speeds[i] = speeds[i] * signs[i];

    // Because we are adding and motors only take values between
    // [-1,1] we may need to normalize them.
    //Find maximum value for speed normalization
    float max_speed = abs(speeds[0]);
    for(int i=1; i<speeds.length; i++) {
        max_speed = max(max_speed, speeds[i]);
    }

    // If and only if the maximum is outside of the range we want it to be,
    // normalize all the other speeds based on the given speed value.
    if (max_speed > 1)
        for (int i=0; i<speeds.length; i++) {
            speeds[i] /= max_speed;
        }
            
    
    this->front_left_motor.set_speed(speeds[0]);
    this->front_right_motor.set_speed(speeds[1]);
    this->back_left_motor.set_speed(speeds[2]);
    this->back_right_motor.set_speed(speeds[3]);
}

float get_left_distance_to_target() {
    return this->target_left_centimeters - this->last_measured_left_centimeters;
}

float get_back_distance_to_target() {
    return this->target_back_centimeters - this->last_measured_back_centimeters;
}

float get_distance_to_target_location() {
    return sqrt(
        sq(this->get_left_distance_to_target()) +
        sq(this->get_back_distance_to_target())
    );
}
float get_degrees_to_target_orientation() {
    return this->target_orientation_degrees - this->last_measured_orientation_degrees;
}

void update_measurements() {
    float new_left_centimeters = (float)this->left_ultrasonic.read();
    float new_back_centimeters = (float)this->right_ultrasonic.read();
    float new_orientation_degrees = magnetometer.getHeadingDegrees() - this->reference_zero_orientation;
    if (abs(new_left_centimeters - this->last_measured_))
}

bool update_towards_target_location(bool update_left = true, bool update_back = true, bool update_orientation = true) {
    this->update_measurements();
    float left_distance_to_target = this->get_left_distance_to_target();
    float back_distance_to_target = this->get_back_distance_to_target();
    float degrees_to_target_orientation = this->get_degrees_to_target_orientation();
    if (!update_left|| left_distance_to_target < this->stop_left_centimeters) {
        left_distance_to_target = 0;
    }
    if (!update_back || back_distance_to_target < this->stop_back_centimeters) {
        back_distance_to_target = 0;
    }
    if (!update_orientation || back_distance_to_target < this->target_orientation_degrees) {
        degrees_to_target_orientation = 0;
    }
    float max_distance = max(back_distance_to_target, left_distance_to_target);
    float drive = back_distance_to_target / max_distance;
    float strafe = left_distance_to_target / max_distance;
    float twist = degrees_to_target_orientation;
    if (back_distance_to_target < this->begin_linear_slowdown_back_centimeters) {
        drive = drive * (back_distance_to_target - this->begin_linear_slowdown_back_centimeters)
                        / (this->begin_linear_slowdown_back_centimeters - this->stop_back_centimeters);
    }
    if (left_distance_to_target < this->begin_linear_slowdown_left_centimeters) {
        drive = drive * (left_distance_to_target - this->begin_linear_slowdown_left_centimeters)
                        / (this->begin_linear_slowdown_left_centimeters - this->stop_left_centimeters);
    }
    if (degrees_to_target_orientation < this->begin_linear_slowdown_degrees) {
        drive = drive * (left_distance_to_target - this->begin_linear_slowdown_left_centimeters)
                        / (this->begin_linear_slowdown_left_centimeters - this->stop_left_centimeters);
    }
    this->set_movement(drive, strafe, twist);
    return drive == 0 && twist == 0 &&  strafe == 0;
}

float get_last_measured_left_centimeters() {
    return this->last_measured_left_centimeters;
}
float get_last_measured_back_centimeters() {
    return this->last_measured_back_centimeters;
}
float get_last_measured_orientation_degrees(){
    return this->last_measured_orientation_degrees;
}
