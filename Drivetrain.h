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
      double reference_zero_orientation,
      double max_allowed_back_centimeters_change = 3.0, double max_allowed_left_centimeters_change = 3.0, double max_allowed_orientation_degrees_change = 5.0,
      double begin_linear_slowdown_back_centimeters = 5.0, double stop_back_centimeters = 1.0,
      double begin_linear_slowdown_left_centimeters = 5.0, double stop_left_centimeters = 1.0,
      double begin_linear_slowdown_degrees = 10.0, double stop_degrees = 2.0,
      double twist_divisor = 20.0
    )
    : front_left_motor(front_left_motor), front_right_motor(front_right_motor), back_left_motor(back_left_motor), back_right_motor(back_right_motor),
      left_ultrasonic(left_ultrasonic), back_ultrasonic(back_ultrasonic),
      reference_zero_orientation(reference_zero_orientation),
      begin_linear_slowdown_back_centimeters(begin_linear_slowdown_back_centimeters), stop_back_centimeters(stop_back_centimeters),
      begin_linear_slowdown_left_centimeters(begin_linear_slowdown_left_centimeters), stop_left_centimeters(stop_left_centimeters),
      begin_linear_slowdown_degrees(begin_linear_slowdown_degrees), stop_degrees(stop_degrees), magnetometer(52, 12), twist_divisor(twist_divisor)
      {}

    void initialize(
      double magnetometer_min_x = 0,
      double magnetometer_max_x = 0,
      double magnetometer_min_y = 0,
      double magnetometer_max_y = 0,
      double magnetometer_min_z = 0,
      double magnetometer_max_z = 0
    );
    double degrees_atan2(double x1, double x2);
    void set_target_location(double left_centimeters, double back_centimeters, double orientation_degrees);
    void set_movement(double drive, double strafe, double twist, bool heading_correction = true);
    void update_measurements();
    // returns true if already stopped at target location
    // not passing tolerances will just use default stop values
    bool update_towards_target_location(
      bool update_left = true,
      bool update_back = true,
      bool update_orientation = true,
      double back_centimeters_tolerance = -1.0,
      double left_centimeters_tolerance = -1.0,
      double orientation_degrees_tolerance = -1.0
    );
    
    double get_last_measured_left_centimeters();
    double get_last_measured_back_centimeters();
    double get_last_measured_orientation_degrees();

    double get_left_distance_to_target_location();
    double get_back_distance_to_target_location();
    double get_degrees_to_target_orientation();

    Magnetometer magnetometer;
  private:
    Motor front_left_motor;
    Motor front_right_motor;
    Motor back_left_motor;
    Motor back_right_motor;
    
    Ultrasonic left_ultrasonic;
    Ultrasonic back_ultrasonic;
    
    double reference_zero_orientation = 0;
    double target_back_centimeters = 0;
    double target_left_centimeters = 0;
    double target_orientation_degrees = 0;
    
    double last_measured_back_centimeters = -1000;
    double last_measured_left_centimeters = -1000;
    double last_measured_orientation_degrees = -1000;
    
    double max_allowed_back_centimeters_change;
    double max_allowed_left_centimeters_change;
    double max_allowed_orientation_degrees_change;

    double begin_linear_slowdown_back_centimeters = 5.0;
    double stop_back_centimeters = 1.0;
    double begin_linear_slowdown_left_centimeters = 5.0;
    double stop_left_centimeters = 1.0;
    double begin_linear_slowdown_degrees = 100.0;
    double stop_degrees = 30.0;
    double twist_divisor = 20.0;
    double max_twist = 0.2;
};