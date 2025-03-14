#include <Arduino.h>
#include "Drivetrain.h"
#include "ArduinoLog.h"
#include <Wire.h>
#include <Servo.h>

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

const int LEFT_ULTRASONIC_TRIG_PIN = A0;
const int LEFT_ULTRASONIC_ECHO_PIN = A3;

const int BACK_ULTRASONIC_TRIG_PIN = A0;
const int BACK_ULTRASONIC_ECHO_PIN = A1;

//const int DUMP_SERVO_PIN = A2;

const double REFERENCE_ZERO_ORIENTATION = 164;
const double MAX_ALLOWED_BACK_CENTIMETERS_CHANGE = 100.0;
const double MAX_ALLOWED_LEFT_CENTIMETERS_CHANGE = 100.0;
const double MAX_ALLOWED_ORIENTATION_DEGREES_CHANGE = 360.0;

const double BEGIN_LINEAR_SLOWDOWN_BACK_CENTIMETERS = 15.0;
const double STOP_BACK_CENTIMETERS = 3.0;

const double BEGIN_LINEAR_SLOWDOWN_LEFT_CENTIMETERS = 15.0;
const double STOP_LEFT_CENTIMETERS = 3.0;

const double BEGIN_LINEAR_SLOWDOWN_DEGREES = 50;
const double STOP_DEGREES = 5.0;
const int STOP_DELAY = 500;
const int RUN_DELAY = 1500;
const double TARGET_ORIENTATION_DEGREES = 0.0;

const double MAGNETOMETER_MIN_X = -9345.20;
const double MAGNETOMETER_MAX_X = 6039.00;
const double MAGNETOMETER_MIN_Y = -21801.40;
const double MAGNETOMETER_MAX_Y = -4426.16;
const double MAGNETOMETER_MIN_Z = 3118.32;
const double MAGNETOMETER_MAX_Z = 6246.40;

const double STARTING_POSITION_CENTIMETERS_LEFT = 5.0;
const double STARTING_POSITION_CENTIMETERS_BACK = 10.0;

const double IGNITER_CENTIMETERS_LEFT = 3.0;
const double IGNITER_CENTIMETERS_BACK = 60.0;

const double MID_MIDPOINT_CENTIMETERS_LEFT = 75.0;
const double MID_MIDPOINT_CENTIMETERS_BACK = 60.0;
const unsigned long MID_MIDPOINT_TIMEOUT_MILLIS = 3000;

const double RIGHT_MIDPOINT_CENTIMETERS_LEFT = 150.0;
const double RIGHT_MIDPOINT_CENTIMETERS_BACK = 60.0;
const unsigned long RIGHT_MIDPOINT_TIMEOUT_MILLIS = 3000;

const double TOP_RIGHT_CENTIMETERS_LEFT = 150.0;
const double TOP_RIGHT_CENTIMETERS_BACK = 85.0;

const double DUMPING_CENTIMETERS_LEFT = 3.0;
const double DUMPING_CENTIMETERS_BACK = 85.0;
const unsigned long MOVING_TO_DUMPING_TIMEOUT_MILLIS = 5000;

const double CUSTOMER_WINDOW_CENTIMETERS_LEFT = 190.0;
const double CUSTOMER_WINDOW_CENTIMETERS_BACK = 15.0;

const double CUSTOMER_WINDOW_WAITING_MILLIS = 5000;

const double DUMPING_SERVO_DUMP_ANGLE = 60;
const double DUMPING_SERVO_NOT_DUMPING_ANGLE = 180;
const double DUMPING_SERVO_DELAY_MILLIS = 5000;

const double STOP_AT_CUSTOMER_WINDOW_MILLIS = 1000;

const bool CORRECT_ORIENTATION_WHILE_MOVING = true;
const bool USE_HEADING_CORRECTION_WHILE_MOVING = true;


typedef enum {
    STATE_ORIENTING_FORWARD,
    STATE_MOVING_TO_IGNITER,
    STATE_MOVING_TO_RIGHT_MIDPOINT,
    STATE_MOVING_TO_MID_MIDPOINT,
    STATE_MOVING_TO_TOP_RIGHT,
    STATE_MOVING_TO_DUMP,
    STATE_DUMPING,
    STATE_MOVING_TO_RIGHT_MIDPOINT_THEN_CUSTOMER_WINDOW,
    STATE_MOVING_TO_CUSTOMER_WINDOW,
    STATE_STOPPED_AT_CUSTOMER_WINDOW,
} States_t;

States_t state;
long state_start_millis = 0;

Motor front_left_motor(FRONT_LEFT_MOTOR_DIRECTION_PIN_0, FRONT_LEFT_MOTOR_DIRECTION_PIN_1, FRONT_LEFT_MOTOR_PWM_PIN, true);
Motor front_right_motor(FRONT_RIGHT_MOTOR_DIRECTION_PIN_0, FRONT_RIGHT_MOTOR_DIRECTION_PIN_1, FRONT_RIGHT_MOTOR_PWM_PIN, true);
Motor back_left_motor(BACK_LEFT_MOTOR_DIRECTION_PIN_0, BACK_LEFT_MOTOR_DIRECTION_PIN_1, BACK_LEFT_MOTOR_PWM_PIN, true);
Motor back_right_motor(BACK_RIGHT_MOTOR_DIRECTION_PIN_0, BACK_RIGHT_MOTOR_DIRECTION_PIN_1, BACK_RIGHT_MOTOR_PWM_PIN, true);

Ultrasonic left_ultrasonic(LEFT_ULTRASONIC_TRIG_PIN, LEFT_ULTRASONIC_ECHO_PIN);
const double BACK_ULTRASONIC_OFFSET_CENTIMETERS = 91.0;
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

void start_moving_from_customer_window() {
    state = STATE_MOVING_TO_DUMP;
}
bool travel_to_location(
  double centimeters_left, double centimeters_back, States_t next_state,
  double target_orientation = 0, 
  double update_left = true, double update_back = true, double update_orientation = true, double heading_correction = true,
  double timeout_millis = -1, double left_centimeters_tolerance = -1, double back_centimeters_tolerance = -1, double orientation_degrees_tolerance = -1
) {
  drivetrain.set_target_location(centimeters_left, centimeters_back, target_orientation);
  bool reached_location = drivetrain.update_towards_target_location(
    update_left, update_back, update_orientation, heading_correction,
    left_centimeters_tolerance, back_centimeters_tolerance, orientation_degrees_tolerance
  );
  if (timeout_millis > 0 && (millis() - state_start_millis > timeout_millis)) {
    state = next_state;
    return true;
  }
  if (reached_location) {
    state = next_state;
  }
  return reached_location;
}

void setup() {
    Serial.begin(115200);
    Log.begin(LOG_LEVEL_SILENT, &Serial);
    
    drivetrain.initialize();
    delay(10000);
    state = STATE_ORIENTING_FORWARD;
    state_start_millis = millis();
}

void log_state() {
    switch(state) {
      case STATE_ORIENTING_FORWARD:
        Log.infoln("STATE_ORIENTING_FORWARD");
        break;
      case STATE_MOVING_TO_IGNITER:
        Log.infoln("STATE_MOVING_TO_IGNITER");
        break;
      case STATE_MOVING_TO_RIGHT_MIDPOINT:
        Log.infoln("STATE_MOVING_TO_RIGHT_MIDPOINT");
        break;
      case STATE_MOVING_TO_TOP_RIGHT:
        Log.infoln("STATE_MOVING_TO_TOP_RIGHT");
        break;
      case STATE_MOVING_TO_DUMP:
        Log.infoln("STATE_MOVING_TO_DUMP");
        break;
      case STATE_DUMPING:
        Log.infoln("STATE_DUMPING");
        break;
      case STATE_MOVING_TO_RIGHT_MIDPOINT_THEN_CUSTOMER_WINDOW:
        Log.infoln("STATE_MOVING_TO_RIGHT_MIDPOINT_THEN_CUSTOMER_WINDOW");
        break;
      case STATE_MOVING_TO_CUSTOMER_WINDOW:
        Log.infoln("STATE_MOVING_TO_CUSTOMER_WINDOW");
        break;
      case STATE_STOPPED_AT_CUSTOMER_WINDOW:
        Log.infoln("STATE_STOPPED_AT_CUSTOMER_WINDOW");
        break;
    }
}

void loop() {
    log_state();
    States_t previous_state = state;
    switch (state) {
        case STATE_ORIENTING_FORWARD:
            travel_to_location(
              STARTING_POSITION_CENTIMETERS_LEFT, STARTING_POSITION_CENTIMETERS_BACK, STATE_MOVING_TO_IGNITER, TARGET_ORIENTATION_DEGREES,
              false, false, true, true
            );
            break;
        case STATE_MOVING_TO_IGNITER:
            travel_to_location(
              IGNITER_CENTIMETERS_LEFT, IGNITER_CENTIMETERS_BACK, STATE_MOVING_TO_MID_MIDPOINT, TARGET_ORIENTATION_DEGREES,
              true, true, CORRECT_ORIENTATION_WHILE_MOVING, USE_HEADING_CORRECTION_WHILE_MOVING
            );
            break;
        case STATE_MOVING_TO_MID_MIDPOINT:
            travel_to_location(
              MID_MIDPOINT_CENTIMETERS_LEFT, MID_MIDPOINT_CENTIMETERS_BACK, STATE_MOVING_TO_RIGHT_MIDPOINT, TARGET_ORIENTATION_DEGREES,
              true, true, CORRECT_ORIENTATION_WHILE_MOVING, USE_HEADING_CORRECTION_WHILE_MOVING, MID_MIDPOINT_TIMEOUT_MILLIS
            );
            break;
        case STATE_MOVING_TO_RIGHT_MIDPOINT:
            travel_to_location(
              RIGHT_MIDPOINT_CENTIMETERS_LEFT, RIGHT_MIDPOINT_CENTIMETERS_BACK, STATE_MOVING_TO_TOP_RIGHT, TARGET_ORIENTATION_DEGREES,
              true, true, CORRECT_ORIENTATION_WHILE_MOVING, USE_HEADING_CORRECTION_WHILE_MOVING, RIGHT_MIDPOINT_TIMEOUT_MILLIS
            );
            break;
        case STATE_MOVING_TO_TOP_RIGHT:
            travel_to_location(
              TOP_RIGHT_CENTIMETERS_LEFT, TOP_RIGHT_CENTIMETERS_BACK, STATE_MOVING_TO_DUMP, TARGET_ORIENTATION_DEGREES,
              false, true, CORRECT_ORIENTATION_WHILE_MOVING, USE_HEADING_CORRECTION_WHILE_MOVING
            );
            break;
        case STATE_MOVING_TO_DUMP:
            travel_to_location(
              DUMPING_CENTIMETERS_LEFT, DUMPING_CENTIMETERS_BACK, STATE_DUMPING, TARGET_ORIENTATION_DEGREES,
              true, true, CORRECT_ORIENTATION_WHILE_MOVING, USE_HEADING_CORRECTION_WHILE_MOVING, MOVING_TO_DUMPING_TIMEOUT_MILLIS
            );
            break;
        case STATE_DUMPING:
            drivetrain.set_movement(0.0, 0.0, 0.0, false);
            //dump_servo.write(DUMPING_SERVO_DUMP_ANGLE);
            //delay(DUMPING_SERVO_DELAY_MILLIS);
            //stop_dump_servo();
            break;
        case STATE_MOVING_TO_RIGHT_MIDPOINT_THEN_CUSTOMER_WINDOW:
            travel_to_location(
              RIGHT_MIDPOINT_CENTIMETERS_LEFT, RIGHT_MIDPOINT_CENTIMETERS_BACK, STATE_MOVING_TO_CUSTOMER_WINDOW, TARGET_ORIENTATION_DEGREES,
              true, true, CORRECT_ORIENTATION_WHILE_MOVING, USE_HEADING_CORRECTION_WHILE_MOVING
            );
            break;
        case STATE_MOVING_TO_CUSTOMER_WINDOW:
            travel_to_location(
              CUSTOMER_WINDOW_CENTIMETERS_LEFT, CUSTOMER_WINDOW_CENTIMETERS_BACK, STATE_MOVING_TO_DUMP, TARGET_ORIENTATION_DEGREES,
              true, true, CORRECT_ORIENTATION_WHILE_MOVING, USE_HEADING_CORRECTION_WHILE_MOVING
            );
            break;
        case STATE_STOPPED_AT_CUSTOMER_WINDOW:
            delay(STOP_AT_CUSTOMER_WINDOW_MILLIS);
            start_moving_from_customer_window();
            break;
        default:
            Serial.println("Unknown state");
            break;
    }
    if (state != previous_state) {
      state_start_millis = millis();
    }
}