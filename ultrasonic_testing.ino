/*
#include "Ultrasonic.h"
Ultrasonic ultrasonic_left(9, 8);
Ultrasonic ultrasonic_back(7, 6);

int left_distance;
int back_distance;

void setup() {
  Serial.begin(9600);
}

void loop() {
  left_distance = ultrasonic_left.read();
  back_distance = ultrasonic_back.read();
  
  Serial.print("Left distance in CM: ");
  Serial.println(left_distance);
  Serial.print("Back distance in CM: ");
  Serial.println(ultrasonic_back);
  delay(1000);
}
*/