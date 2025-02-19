#include "Ultrasonic.h"

/*
 * Pass as a parameter the trigger and echo pin, respectively,
 * or only the signal pin (for sensors 3 pins), like:
 * Ultrasonic ultrasonic(13);
 */
Ultrasonic ultrasonicLeft(9, 8);
Ultrasonic ultrasonicBack(7, 6);

int leftDistance;
int backDistance;

void setup() {
  Serial.begin(9600);
}

void loop() {
  leftDistance = ultrasonicLeft.read();
  backDistance = ultrasonicBack.read();
  
  Serial.print("Left distance in CM: ");
  Serial.println(leftDistance);
  Serial.print("Back distance in CM: ");
  Serial.println(backDistance);
  delay(1000);
}