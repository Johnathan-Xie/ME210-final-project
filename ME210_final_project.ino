#include <Arduino.h>

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

void setup() {
    Serial.begin(9600);

    pinMode(FRONT_LEFT_MOTOR_DIRECTION_PIN_0, OUTPUT);
    pinMode(FRONT_LEFT_MOTOR_DIRECTION_PIN_1, OUTPUT);
    pinMode(FRONT_LEFT_MOTOR_PWM_PIN, OUTPUT);

    pinMode(FRONT_RIGHT_MOTOR_DIRECTION_PIN_0, OUTPUT);
    pinMode(FRONT_RIGHT_MOTOR_DIRECTION_PIN_1, OUTPUT);
    pinMode(FRONT_RIGHT_MOTOR_PWM_PIN, OUTPUT);

    pinMode(BACK_LEFT_MOTOR_DIRECTION_PIN_0, OUTPUT);
    pinMode(BACK_LEFT_MOTOR_DIRECTION_PIN_1, OUTPUT);
    pinMode(BACK_LEFT_MOTOR_PWM_PIN, OUTPUT);

    pinMode(BACK_RIGHT_MOTOR_DIRECTION_PIN_0, OUTPUT);
    pinMode(BACK_RIGHT_MOTOR_DIRECTION_PIN_1, OUTPUT);
    pinMode(BACK_RIGHT_MOTOR_PWM_PIN, OUTPUT);
}

void write_motor(int direction_pin_0, int direction_pin_1, int pwm_pin, int pwm_value) {
  if (pwm_value < 0) {
    digitalWrite(direction_pin_0, LOW);
    digitalWrite(direction_pin_1, HIGH);

    Serial.print(direction_pin_0);
    Serial.println(" set to LOW");
    Serial.print(direction_pin_1);
    Serial.println(" set to HIGH");
  } else {
    digitalWrite(direction_pin_0, HIGH);
    digitalWrite(direction_pin_1, LOW);

    Serial.print(direction_pin_0);
    Serial.println(" set to HIGH");
    Serial.print(direction_pin_1);
    Serial.println(" set to LOW");
  }
  analogWrite(pwm_pin, abs(pwm_value));
  Serial.print(pwm_pin);
  Serial.print(" set to ");
  Serial.println(abs(pwm_value));
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        int colon_index = input.indexOf(':');
        if (colon_index != -1) {
            String speed_str = input.substring(colon_index + 1);
            speed_str.trim();
            
            int speed = speed_str.toInt();
            
            speed = constrain(speed, -255, 255);
            String motor_str = input.substring(0, colon_index);
            if (motor_str == "front_left") {
              write_motor(FRONT_LEFT_MOTOR_DIRECTION_PIN_0, FRONT_LEFT_MOTOR_DIRECTION_PIN_1, FRONT_LEFT_MOTOR_PWM_PIN, speed);
            } else if (motor_str == "front_right") {
              write_motor(FRONT_RIGHT_MOTOR_DIRECTION_PIN_0, FRONT_RIGHT_MOTOR_DIRECTION_PIN_1, FRONT_RIGHT_MOTOR_PWM_PIN, speed);
            } else if (motor_str == "back_left") {
              write_motor(BACK_LEFT_MOTOR_DIRECTION_PIN_0, BACK_LEFT_MOTOR_DIRECTION_PIN_1, BACK_LEFT_MOTOR_PWM_PIN, speed);
            } else if (motor_str == "back_right") {
              write_motor(BACK_RIGHT_MOTOR_DIRECTION_PIN_0, BACK_RIGHT_MOTOR_DIRECTION_PIN_1, BACK_RIGHT_MOTOR_PWM_PIN, speed);
            } else{
              Serial.println("Invalid motor input");
            }
        } else {
          Serial.println("Invalid input");
        }
    }
}