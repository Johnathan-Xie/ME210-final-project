#pragma once

#include "Motor.h"
#include "Ultrasonic.h"
#include "Magnetometer.h"
#include "DFRobot_QMC5883.h"
#include "IMU.h"

// Potentially add provision for integrating for location
class Drivetrain {
  public:
    Drivetrain(
      Motor& front_left_motor, Motor& front_right_motor, Motor& back_left_motor, Motor& back_right_motor,
      Ultrasonic& left_ultrasonic, Ultrasonic& back_ultrasonic,
      double reference_zero_orientation,
      double max_allowed_back_centimeters_change = 20.0, double max_allowed_left_centimeters_change = 20.0, double max_allowed_orientation_degrees_change = 5.0,
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
      begin_linear_slowdown_degrees(begin_linear_slowdown_degrees), stop_degrees(stop_degrees), heading_indicator(), twist_divisor(twist_divisor)
      {}

    void initialize();
    double degrees_atan2(double x1, double x2);
    void set_target_location(double left_centimeters, double back_centimeters, double orientation_degrees);
    void set_movement(double drive, double strafe, double twist, bool heading_correction = true);
    void update_measurements(
      bool update_left = true,
      bool update_back = true,
      bool update_orientation = true
    );

    // returns true if already stopped at target location
    // not passing tolerances will just use default stop values
    bool update_towards_target_location(
      bool update_left = true,
      bool update_back = true,
      bool update_orientation = true,
      bool heading_correction = true,
      double left_centimeters_tolerance = -1.0,
      double back_centimeters_tolerance = -1.0,
      double orientation_degrees_tolerance = -1.0
    );
    double clip_max(double value, double max_value);
    double get_last_measured_left_centimeters();
    double get_last_measured_back_centimeters();
    double get_last_measured_orientation_degrees();

    double get_left_distance_to_target_location();
    double get_back_distance_to_target_location();
    double get_degrees_to_target_orientation();

    IMU heading_indicator;
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
    
    double max_drive = 0.5;
    double max_strafe = 0.5;
    double max_twist = 0.2;

    double left_centimeters_degrees_drift = 0;
    //double min_twist = 0.2;
    //double min_drive = 0.2;
    //double min_strafe = 0.2;
    
    double max_degrees_error_to_still_move = 30.0;
};