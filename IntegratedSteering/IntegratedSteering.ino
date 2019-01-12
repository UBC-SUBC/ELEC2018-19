#include <Wire.h>

#include <MPU6050.h>

#include <Servo.h>

Servo xservo;
Servo yservo;

const int joyXPin = A0;
const int joyYPin = A1;
const int xservoPin = 9;
const int yservoPin = 10;
const int manualSwitchPin = 11; 

int joystick_y_in, joystick_x_in, manualSwitch;

MPU6050 sensor ;

int16_t ax, ay, az ;

int16_t gx, gy, gz ;

void setup(){

  yservo.attach(yservoPin);
  xservo.attach(xservoPin);

  pinMode(manualSwitchPin, INPUT_PULLUP);
  
  Wire.begin ( );

  Serial.begin  (9600);

  Serial.println  ( "Initializing the sensor" );

  sensor.initialize ( );

  Serial.println (sensor.testConnection ( ) ? "Successfully Connected" : "Connection failed");

  delay (1000);

  Serial.println ( "Taking Values from the sensor" );

  delay (1000);

}


void loop(){

  // Since x is not controlled by the IMU, we can put it outside of the for loop.
  readInput();
  xservo.write(joystick_x_in);
  Serial.print("X:");
  Serial.print(joystick_x_in);
  
  manualSwitch = digitalRead(manualSwitchPin);
  if (manualSwitch == 1) {
    //Serial.println("Manual steering is on!");
    yservo.write(joystick_y_in);

    Serial.print(" ManualY:");
    Serial.println(joystick_y_in);
  }
  else {
    sensor.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);

    ax = map (ax, -17000, 17000, 0, 180) ;

    Serial.print("AutomatedY:");
    Serial.println (ax);
  
    yservo.write (ax);
  
    //delay (200);
  }
  delay(100);

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
