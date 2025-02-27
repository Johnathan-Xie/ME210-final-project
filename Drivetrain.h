#pragma once

#include "Motor.h"
#include "Ultrasonic.h"
#include "Magnetometer.h"
#include "DFRobot_QMC5883.h"

// Potentially add provision for integrating for location
class Drivetrain {
  public:
    Drivetrain(
      Motor& front_left_motor, Motor& front_right_motor, Motor& back_left_motor, Motor& back_right_motor,
      Ultrasonic& left_ultrasonic, Ultrasonic& back_ultrasonic,
      float reference_zero_orientation,
      float max_allowed_back_centimeters_change = 3.0, float max_allowed_left_centimeters_change = 3.0, float max_allowed_orientation_degrees_change = 5.0,
      float begin_linear_slowdown_back_centimeters = 5.0, float stop_back_centimeters = 1.0,
      float begin_linear_slowdown_left_centimeters = 5.0, float stop_left_centimeters = 1.0,
      float begin_linear_slowdown_degrees = 10.0, float stop_degrees = 2.0,
      float twist_divisor = 20.0
    )
    : front_left_motor(front_left_motor), front_right_motor(front_right_motor), back_left_motor(back_left_motor), back_right_motor(back_right_motor),
      left_ultrasonic(left_ultrasonic), back_ultrasonic(back_ultrasonic),
      reference_zero_orientation(reference_zero_orientation),
      begin_linear_slowdown_back_centimeters(begin_linear_slowdown_back_centimeters), stop_back_centimeters(stop_back_centimeters),
      begin_linear_slowdown_left_centimeters(begin_linear_slowdown_left_centimeters), stop_left_centimeters(stop_left_centimeters),
      begin_linear_slowdown_degrees(begin_linear_slowdown_degrees), stop_degrees(stop_degrees), magnetometer(52, 12), twist_divisor(twist_divisor)
      {}

    void initialize();
    void set_target_location(float left_centimeters, float back_centimeters, float orientation_degrees);
    void set_movement(float drive, float strafe, float twist, bool heading_correction = true);
    void update_measurements();
    // returns true if already stopped at target location
    // not passing tolerances will just use default stop values
    bool update_towards_target_location(
      bool update_left = true,
      bool update_back = true,
      bool update_orientation = true,
      float back_centimeters_tolerance = -1.0,
      float left_centimeters_tolerance = -1.0,
      float orientation_degrees_tolerance = -1.0
    );
    
    float get_last_measured_left_centimeters();
    float get_last_measured_back_centimeters();
    float get_last_measured_orientation_degrees();

    float get_left_distance_to_target_location();
    float get_back_distance_to_target_location();
    float get_degrees_to_target_orientation();
  private:
    Motor front_left_motor;
    Motor front_right_motor;
    Motor back_left_motor;
    Motor back_right_motor;
    Magnetometer magnetometer;
    Ultrasonic left_ultrasonic;
    Ultrasonic back_ultrasonic;
    
    float reference_zero_orientation = 0;
    float target_back_centimeters = 0;
    float target_left_centimeters = 0;
    float target_orientation_degrees = 0;
    
    float last_measured_back_centimeters = -1000;
    float last_measured_left_centimeters = -1000;
    float last_measured_orientation_degrees = -1000;
    
    float max_allowed_back_centimeters_change;
    float max_allowed_left_centimeters_change;
    float max_allowed_orientation_degrees_change;

    float begin_linear_slowdown_back_centimeters = 5.0;
    float stop_back_centimeters = 1.0;
    float begin_linear_slowdown_left_centimeters = 5.0;
    float stop_left_centimeters = 1.0;
    float begin_linear_slowdown_degrees = 100.0;
    float stop_degrees = 30.0;
    float twist_divisor = 20.0;
    float max_twist = 0.2;
};