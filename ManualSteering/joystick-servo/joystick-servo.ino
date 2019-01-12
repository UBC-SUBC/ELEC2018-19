#include <Servo.h>

const int joyXPin = A0;
const int joyYPin = A1;
const int xservoPin = 9;
const int yservoPin = 10;
const int manualSwitchPin = 11; 

int joystick_y_in;
int joystick_x_in;
int manualSwitch;

Servo xservo;
Servo yservo;

void setup() {
  Serial.begin(9600);
  delay(200);
  xservo.attach(xservoPin);
  yservo.attach(yservoPin);
  pinMode(manualSwitchPin, INPUT_PULLUP);
}

void loop() {
  manualSwitch = digitalRead(manualSwitchPin);
  if (manualSwitch == 1){
    //Serial.println("Manual steering is on!");
    readInput();
    xservo.write(joystick_x_in);
    yservo.write(joystick_y_in);
    Serial.print("X:");
    Serial.print(joystick_x_in);
    Serial.print(" Y:");
    Serial.println(joystick_y_in);
  }
  else Serial.println("Manual steering is off.");
  delay(30);
}

void readInput() {
  joystick_x_in = analogRead(joyXPin);
  joystick_y_in = analogRead(joyYPin);

  // Note: Changed to normal Arduino map function because betterMap is somehow wrong (I didn't do the math to figure out why). Normal map function seems to work well though. --Ryan Meshulam
  joystick_x_in = map(joystick_x_in, 101, 922, 0, 180);
  joystick_y_in = map(joystick_y_in, 101, 923, 0, 180);
  
  if (joystick_x_in < 97 && joystick_x_in > 83) {
    joystick_x_in = 90;
  }
  if (joystick_y_in < 97 && joystick_y_in > 83) {
    joystick_y_in = 90;
  }
}

int betterMap(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}
