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

const int DUMP_SERVO_PIN = A2;

const double REFERENCE_ZERO_ORIENTATION = -120;
const double MAX_ALLOWED_BACK_CENTIMETERS_CHANGE = 100.0;
const double MAX_ALLOWED_LEFT_CENTIMETERS_CHANGE = 100.0;
const double MAX_ALLOWED_ORIENTATION_DEGREES_CHANGE = 360.0;

const double BEGIN_LINEAR_SLOWDOWN_BACK_CENTIMETERS = 30.0;
const double STOP_BACK_CENTIMETERS = 3.0;

const double BEGIN_LINEAR_SLOWDOWN_LEFT_CENTIMETERS = 30.0;
const double STOP_LEFT_CENTIMETERS = 3.0;

const double BEGIN_LINEAR_SLOWDOWN_DEGREES = 50;
const double STOP_DEGREES = 10.0;
const int STOP_DELAY = 500;
const int RUN_DELAY = 1500;
const double TARGET_ORIENTATION_DEGREES = 0.0;

const double MAGNETOMETER_MIN_X = -9345.20;
const double MAGNETOMETER_MAX_X = 6039.00;
const double MAGNETOMETER_MIN_Y = -21801.40;
const double MAGNETOMETER_MAX_Y = -4426.16;
const double MAGNETOMETER_MIN_Z = 3118.32;
const double MAGNETOMETER_MAX_Z = 6246.40;

const double STARTING_POSITION_CENTIMETERS_LEFT = 3.0;
const double STARTING_POSITION_CENTIMTERS_BACK = 10.0;

const double IGNITER_CENTIMETERS_LEFT = 3.0;
const double IGNITER_CENTIMETERS_BACK = 64.0;

const double MID_MIDPOINT_CENTIMETERS_LEFT = 100.0;
const double MID_MIDPOINT_CENTIMETERS_BACK = 64.0;

const double RIGHT_MIDPOINT_CENTIMETERS_LEFT = 200.0;
const double RIGHT_MIDPOINT_CENTIMETERS_BACK = 64.0;

const double TOP_RIGHT_CENTIMETERS_LEFT = 200.0;
const double TOP_RIGHT_CENTIMETERS_BACK = 89.0;

const double TOP_LEFT_RIGHT_OF_BUCKET_CENTIMETERS_LEFT = 54.0;
const double TOP_LEFT_RIGHT_OF_BUCKET_CENTIMETERS_BACK = 89.0;

const double MIDPOINT_BEFORE_DUMP_POSITION_CENTIMETERS_LEFT = 54.0;
const double MIDPOINT_BEFORE_DUMP_POSITION_CENTIMETERS_BACK = 44.0;

const double DUMP_POSITION_CENTIMETERS_LEFT = 22.0;
const double DUMP_POSITION_CENTIMETERS_BACK = 44.0;
const bool USE_MAGNETOMETER_WHILE_MOVING_ORIENTATION = true;
const bool USE_MAGNETOMETER_FOR_HEADING_CORRECTION_WHILE_MOVING = false;
typedef enum {
    STATE_ORIENTING_FORWARD,
    STATE_MOVING_TO_IGNITER,
    STATE_MOVING_TO_RIGHT_MIDPOINT,
    STATE_MOVING_TO_MID_MIDPOINT,
    STATE_MOVING_TO_TOP_RIGHT,
    STATE_MOVING_TO_TOP_LEFT_RIGHT_OF_BUCKET,
    STATE_MOVING_TO_MIDPOINT_BEFORE_DUMP_POSITION,
    STATE_MOVING_TO_DUMP_POSITION,
    STATE_DUMPING
} States_t;

States_t state;

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

void setup() {
    Serial.begin(9600);
    Log.begin(LOG_LEVEL_SILENT, &Serial);
    
    drivetrain.initialize(
      MAGNETOMETER_MIN_X,
      MAGNETOMETER_MAX_X,
      MAGNETOMETER_MIN_Y,
      MAGNETOMETER_MAX_Y,
      MAGNETOMETER_MIN_Z,
      MAGNETOMETER_MAX_Z
    );
    state = STATE_ORIENTING_FORWARD;
    drivetrain.magnetometer.compass.enableCalibration(false);
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
      case STATE_MOVING_TO_TOP_LEFT_RIGHT_OF_BUCKET:
        Log.infoln("STATE_MOVING_TO_TOP_LEFT_RIGHT_OF_BUCKET");
        break;
    }
}

void loop() {
    log_state();
    switch (state) {
        case STATE_ORIENTING_FORWARD:
            drivetrain.set_target_location(STARTING_POSITION_CENTIMETERS_LEFT, STARTING_POSITION_CENTIMTERS_BACK, TARGET_ORIENTATION_DEGREES);
            if (drivetrain.update_towards_target_location(false, false, true, false)) {
                state = STATE_MOVING_TO_IGNITER;
            }
            break;
        case STATE_MOVING_TO_IGNITER:
            drivetrain.set_target_location(IGNITER_CENTIMETERS_LEFT, IGNITER_CENTIMETERS_BACK, TARGET_ORIENTATION_DEGREES);
            if (drivetrain.update_towards_target_location(true, true, USE_MAGNETOMETER_WHILE_MOVING_ORIENTATION, USE_MAGNETOMETER_FOR_HEADING_CORRECTION_WHILE_MOVING)) {
                state = STATE_MOVING_TO_MID_MIDPOINT;
            }
            break;
        case STATE_MOVING_TO_MID_MIDPOINT:
            drivetrain.set_target_location(MID_MIDPOINT_CENTIMETERS_LEFT, MID_MIDPOINT_CENTIMETERS_BACK, TARGET_ORIENTATION_DEGREES);
            if (drivetrain.update_towards_target_location(true, true, USE_MAGNETOMETER_WHILE_MOVING_ORIENTATION, USE_MAGNETOMETER_FOR_HEADING_CORRECTION_WHILE_MOVING)) {
                state = STATE_MOVING_TO_RIGHT_MIDPOINT;
            }
        case STATE_MOVING_TO_RIGHT_MIDPOINT:
            drivetrain.set_target_location(RIGHT_MIDPOINT_CENTIMETERS_LEFT, RIGHT_MIDPOINT_CENTIMETERS_BACK, TARGET_ORIENTATION_DEGREES);
            if (drivetrain.update_towards_target_location(true, true, USE_MAGNETOMETER_WHILE_MOVING_ORIENTATION, USE_MAGNETOMETER_FOR_HEADING_CORRECTION_WHILE_MOVING)) {
                state = STATE_MOVING_TO_TOP_RIGHT;
            }
            break;
        case STATE_MOVING_TO_TOP_RIGHT:
            drivetrain.set_target_location(TOP_RIGHT_CENTIMETERS_LEFT, TOP_RIGHT_CENTIMETERS_BACK, TARGET_ORIENTATION_DEGREES);
            if (drivetrain.update_towards_target_location(true, true, USE_MAGNETOMETER_WHILE_MOVING_ORIENTATION, USE_MAGNETOMETER_FOR_HEADING_CORRECTION_WHILE_MOVING)) {
                state = STATE_MOVING_TO_TOP_LEFT_RIGHT_OF_BUCKET;
            }
            break;
        case STATE_MOVING_TO_TOP_LEFT_RIGHT_OF_BUCKET:
            drivetrain.set_target_location(TOP_LEFT_RIGHT_OF_BUCKET_CENTIMETERS_LEFT, TOP_LEFT_RIGHT_OF_BUCKET_CENTIMETERS_BACK, TARGET_ORIENTATION_DEGREES);
            if (drivetrain.update_towards_target_location(true, true, USE_MAGNETOMETER_WHILE_MOVING_ORIENTATION, USE_MAGNETOMETER_FOR_HEADING_CORRECTION_WHILE_MOVING)) {

            }
            break;
        default:
            Serial.println("Unknown state");
        
    }
}