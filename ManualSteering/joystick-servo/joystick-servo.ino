#include <Servo.h>
// Uncomment when using Duemilanove
const int joyXPin = A0;
const int joyYPin = A1;
const int xservoPin = 9;
const int yservoPin = 10;

// Uncomment when using MEGA
/*
const int joyXPin = 14;
const int joyYPin = 15;
*/
int joystick_y_in;
int joystick_x_in;

Servo myservo;
Servo yservo;


void setup() {
  Serial.begin(9600);
  delay(200);
  myservo.attach(xservoPin);
  yservo.attach(yservoPin);
}

void loop() {
  readInput();
  myservo.write(joystick_x_in);
  yservo.write(joystick_y_in);
  Serial.print("X:");
  Serial.print(joystick_x_in);
  Serial.print(" Y:");
  Serial.println(joystick_y_in);
  delay(30);
}

void readInput() {
  joystick_x_in = analogRead(joyXPin);
  joystick_y_in = analogRead(joyYPin);

  //joystick_x_in = betterMap(joystick_x_in, 101, 922, 60, 120);
  //joystick_y_in = betterMap(joystick_y_in, 101, 923, 60, 120);
  joystick_x_in = betterMap(joystick_x_in, 101, 922, 0, 180);
  joystick_y_in = betterMap(joystick_y_in, 101, 923, 0, 180);
  if (joystick_x_in < 97 && joystick_x_in > 83) {
    joystick_x_in = 90;
  }
  if (joystick_y_in < 97 && joystick_y_in > 83) {
    joystick_y_in = 90;
  }
}

long betterMap(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}
