#include <Arduino.h>
#include "Drivetrain.h"
#include "ArduinoLog.h"
#include <Wire.h>
const int MAGNETOMETER_DECLINATION_DEGS = 12;
const int MAGNETOMETER_DECLINATION_MINS = 52;
const char MAGNETOMETER_DECLINATION_DIR = 'E';

const int FRONT_LEFT_MOTOR_DIRECTION_PIN_0 = 7;
const int FRONT_LEFT_MOTOR_DIRECTION_PIN_1 = 8;
const int FRONT_LEFT_MOTOR_PWM_PIN = 10;

const int FRONT_RIGHT_MOTOR_DIRECTION_PIN_0 = 12;
const int FRONT_RIGHT_MOTOR_DIRECTION_PIN_1 = 13;
const int FRONT_RIGHT_MOTOR_PWM_PIN = 11;

const int BACK_LEFT_MOTOR_DIRECTION_PIN_0 = 4;
const int BACK_LEFT_MOTOR_DIRECTION_PIN_1 = 2;
const int BACK_LEFT_MOTOR_PWM_PIN = 9;

const int BACK_RIGHT_MOTOR_DIRECTION_PIN_0 = 6;
const int BACK_RIGHT_MOTOR_DIRECTION_PIN_1 = 5;
const int BACK_RIGHT_MOTOR_PWM_PIN = 3;

const int LEFT_ULTRASONIC_TRIG_PIN = A2;
const int LEFT_ULTRASONIC_ECHO_PIN = A3;

const int BACK_ULTRASONIC_TRIG_PIN = A0;
const int BACK_ULTRASONIC_ECHO_PIN = A1;

const double REFERENCE_ZERO_ORIENTATION = 70.0;
const double MAX_ALLOWED_BACK_CENTIMETERS_CHANGE = 100.0;
const double MAX_ALLOWED_LEFT_CENTIMETERS_CHANGE = 100.0;
const double MAX_ALLOWED_ORIENTATION_DEGREES_CHANGE = 360.0;

const double BEGIN_LINEAR_SLOWDOWN_BACK_CENTIMETERS = 30.0;
const double STOP_BACK_CENTIMETERS = 5.0;

const double BEGIN_LINEAR_SLOWDOWN_LEFT_CENTIMETERS = 30.0;
const double STOP_LEFT_CENTIMETERS = 5.0;

const double BEGIN_LINEAR_SLOWDOWN_DEGREES = 60.0;
const double STOP_DEGREES = 15.0;
const int STOP_DELAY = 500;
const int RUN_DELAY = 1500;
const float SQUARE_SPEED = 0.4;
const double TARGET_ORIENTATION_DEGREES = 0.0;

const double MAGNETOMETER_MIN_X = -15010.8;
const double MAGNETOMETER_MAX_X = 12444.00;
const double MAGNETOMETER_MIN_Y = 756.40;
const double MAGNETOMETER_MAX_Y = 21435.40;
const double MAGNETOMETER_MIN_Z = 46838.24;
const double MAGNETOMETER_MAX_Z = 53094.4;

Motor front_left_motor(FRONT_LEFT_MOTOR_DIRECTION_PIN_0, FRONT_LEFT_MOTOR_DIRECTION_PIN_1, FRONT_LEFT_MOTOR_PWM_PIN, false);
Motor front_right_motor(FRONT_RIGHT_MOTOR_DIRECTION_PIN_0, FRONT_RIGHT_MOTOR_DIRECTION_PIN_1, FRONT_RIGHT_MOTOR_PWM_PIN, false);
Motor back_left_motor(BACK_LEFT_MOTOR_DIRECTION_PIN_0, BACK_LEFT_MOTOR_DIRECTION_PIN_1, BACK_LEFT_MOTOR_PWM_PIN, false);
Motor back_right_motor(BACK_RIGHT_MOTOR_DIRECTION_PIN_0, BACK_RIGHT_MOTOR_DIRECTION_PIN_1, BACK_RIGHT_MOTOR_PWM_PIN, false);

Ultrasonic left_ultrasonic(LEFT_ULTRASONIC_TRIG_PIN, LEFT_ULTRASONIC_ECHO_PIN);
const double BACK_ULTRASONIC_OFFSET_CENTIMETERS = 65;
bool BACK_ULTRASONIC_REVERSED = true;
Ultrasonic back_ultrasonic(BACK_ULTRASONIC_TRIG_PIN, BACK_ULTRASONIC_ECHO_PIN, 20000, BACK_ULTRASONIC_REVERSED, BACK_ULTRASONIC_OFFSET_CENTIMETERS);

Drivetrain drivetrain(
  front_left_motor, front_right_motor, back_left_motor, back_right_motor,
  left_ultrasonic, back_ultrasonic,
  REFERENCE_ZERO_ORIENTATION,
  MAX_ALLOWED_BACK_CENTIMETERS_CHANGE, MAX_ALLOWED_LEFT_CENTIMETERS_CHANGE, MAX_ALLOWED_ORIENTATION_DEGREES_CHANGE,
  BEGIN_LINEAR_SLOWDOWN_BACK_CENTIMETERS, STOP_BACK_CENTIMETERS,
  BEGIN_LINEAR_SLOWDOWN_LEFT_CENTIMETERS, STOP_LEFT_CENTIMETERS,
  BEGIN_LINEAR_SLOWDOWN_DEGREES, STOP_DEGREES
);

void setup() {
  Serial.begin(9600);
  Log.begin(LOG_LEVEL_INFO, &Serial);
  
  drivetrain.initialize(
    MAGNETOMETER_MIN_X,
    MAGNETOMETER_MAX_X,
    MAGNETOMETER_MIN_Y,
    MAGNETOMETER_MAX_Y,
    MAGNETOMETER_MIN_Z,
    MAGNETOMETER_MAX_Z
  );
  
  drivetrain.magnetometer.compass.enableCalibration(false);
  //drivetrain.set_target_location(19, 23, TARGET_ORIENTATION_DEGREES);
}
 
void loop() {
  drivetrain.set_movement(SQUARE_SPEED, 0, 0, false);
  delay(RUN_DELAY);
  drivetrain.set_movement(0, 0, 0, false);
  delay(STOP_DELAY);
  drivetrain.set_movement(0, SQUARE_SPEED, 0, false);
  delay(RUN_DELAY);
  drivetrain.set_movement(0, 0, 0, false);
  delay(STOP_DELAY);
  drivetrain.set_movement(-SQUARE_SPEED, 0, 0, false);
  delay(RUN_DELAY);
  drivetrain.set_movement(0, 0, 0, false);
  delay(STOP_DELAY);
  drivetrain.set_movement(0, -SQUARE_SPEED, 0, false);
  delay(RUN_DELAY);
  drivetrain.set_movement(0, 0, 0, false);
  delay(STOP_DELAY);
}