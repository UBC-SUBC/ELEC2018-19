// Elec steering and buoyancy code 2018
// SUBC: the UBC Submarine Design Team

// Libraries for communicating with IMU
#include <Wire.h>
#include <MPU6050.h>
// Steering libraries
#include <Servo.h>
// Buoyancy libraries

// Initialize servos
Servo xservo;
Servo yservo;

// Steering constants
const int joyXPin = A0;
const int joyYPin = A1;
const int xservoPin = 9;
const int yservoPin = 10;
const int manualSwitchPin = 11;
#define MANUAL true
#define AUTOMATED false

// Manual variables and objects
int joystick_y_in, joystick_x_in;
bool manualSwitch = MANUAL;

// IMU variables and objects
MPU6050 sensor;
bool sensorWorking = false;
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup(){

  // Initialize serial monitoring
  Serial.begin(9600);
  
  // Initialize servos
  yservo.attach(yservoPin);
  xservo.attach(xservoPin);

  // Initialize manual control
  pinMode(manualSwitchPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(manualSwitchPin), switchManual, CHANGE);

  // Initialize IMU
  Wire.begin();
  Serial.println( "Initializing the sensor" );
  sensor.initialize ( );
  sensorWorking = sensor.testConnection();
  if (sensorWorking == true){
    Serial.println ("Successfully Connected");
    delay (1000);
    Serial.println ( "Taking Values from the sensor" );
    delay (1000);
  } else {
    Serial.println("Connection failed");
  }
  switchManual();
  
}


void loop(){

  // Since x is not controlled by the IMU, we can put it outside of the for loop.
  readInput();
  xservo.write(joystick_x_in);
  Serial.print("X:");
  Serial.print(joystick_x_in);
  
  if (manualSwitch==AUTOMATED) {
    sensor.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);

    ax = map (ax, -17000, 17000, 0, 180) ;

    Serial.print("AutomatedY:");
    Serial.println (ax);
  
    yservo.write (ax);
  
    //delay (200);
  } else {
    //Serial.println("Manual steering is on!");
    yservo.write(joystick_y_in);

    Serial.print(" ManualY:");
    Serial.println(joystick_y_in);
  }
  
  delay(100);

}

void readInput() {
  joystick_x_in = analogRead(joyXPin);
  joystick_y_in = analogRead(joyYPin);
  //Serial.print("Read inputs");

  // Note: Changed to normal Arduino map function because betterMap is somehow wrong (I didn't do the math to figure out why). Normal map function seems to work well though. --Ryan Meshulam
  joystick_x_in = map(joystick_x_in, 101, 922, 0, 180);
  joystick_y_in = map(joystick_y_in, 101, 923, 0, 180);
  //Serial.print(joystick_x_in);
  //Serial.println(joystick_y_in);
  if (joystick_x_in < 97 && joystick_x_in > 83) {
    joystick_x_in = 90;
  }
  if (joystick_y_in < 97 && joystick_y_in > 83) {
    joystick_y_in = 90;
  }
}

void switchManual(){
  manualSwitch = digitalRead(manualSwitchPin);
  Serial.print("result of digitalread");
  Serial.println(manualSwitch);
  sensorWorking = sensor.testConnection();
  Serial.println(sensorWorking);
  /*if (sensorWorking){
    manualSwitch = digitalRead(manualSwitchPin);
  } else {
    manualSwitch = MANUAL;
  }*/
  manualSwitch=AUTOMATED;
}
