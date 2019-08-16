// SUBC 2018-2019
// Electrical steering code

/*****Libraries*****/
// I2C device communication library
#include <Wire.h>
#include "I2Cdev.h"
// IMU sensor library
#include <MPU6050.h>
// Steering libraries
#include <Servo.h>

/***Arduino pin assignments***/
//Joystick pins
const int joyXPin = A0;
const int joyYPin = A1;
//Servo pins
const int xservoPin = 9;
const int yservoPin = 10;

// Smoothing constants
#define SMOOTHINGERROR 2
#define ORIGINERROR 10

#define SERVORANGE 36

// IMU variables and objects
MPU6050 IMU;
bool IMUWorking = false;
int16_t ax, ay, az, azPrev, oldAX=90;
int16_t gx, gy, gz;
bool AUTO_ON;

// Pin A4 and A5 are reserved for I2C sensors
// We are unsure which is which...

// Initialize servos
Servo xservo;
Servo yservo;

/******MAIN SETUP FUNCTION******/
void setup() {

  // Initialize serial monitoring
  Serial.begin(9600);

  // Initialize servos
  yservo.attach(yservoPin);
  xservo.attach(xservoPin);

  // Default set automated steering on
  AUTO_ON = true;

  // Initialize IMU
  Wire.begin();
  Serial.println( "Initializing the sensor" );
  IMU.initialize ( );
  IMUWorking = IMU.testConnection();
  if (IMUWorking == true){
    Serial.println ("Successfully Connected");
    delay (1000);
    Serial.println ( "Taking Values from the sensor" );
    delay (1000);
  } else {
    Serial.println("Connection failed");
  } 
}

/********MAIN LOOP FUNCTION********/
void loop() {
  int joystick_x_in, joystick_y_in;

  // read the x input from the joystick and use it to control the servocs
  joystick_x_in, joystick_y_in = readInput();
  joystick_x_in=servoLimiter(joystick_x_in);
  moveXServo(joystick_x_in, joystick_y_in);

  // get the position from the IMU and make sure
  IMU.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);

  AUTO_ON = checkAutomatedSteering(ax, ay);

  //if (AUTO_ON) {
  // for right now, simply check if the y reading is out of the range it should be in
  // if it is, then we know auto is not working and should turn it off
  if (ay > 35){
    ax = map (ax, -17000, 21000, 0, 180) ;

      if (ax>90-ORIGINERROR&&ax<90+ORIGINERROR) {
        ax=90;
      }

      else if(ax>oldAX-SMOOTHINGERROR&&ax<oldAX+SMOOTHINGERROR) {
        ax=oldAX;
      }

      ax=servoLimiter(ax);
      Serial.print(" AutomatedY:");
      Serial.print(ax);

      Serial.println("");

      yservo.write (ax);
      oldAX=ax;
  } 
  // if automated starts malfunctioning, let the joystick
  else {
  //Serial.println("Manual steering is on!");
  joystick_y_in=servoLimiter(joystick_y_in);
  yservo.write(joystick_y_in);

  Serial.print(" ManualY:");
  Serial.println(joystick_y_in);
  }
}


/********FUNCTIONS********/
// Filter to check automated steering trends
// Saves 100 iterations of automated steering values and
bool checkAutomatedSteering(int x, int y){
  return (false);
}


// Moves the servo in the x position based on joystick input
void moveXServo(int joystick_x_in, int joystick_y_in){
  xservo.write(joystick_x_in);
  Serial.print("X:");
  Serial.print(joystick_x_in);

  return;
}

// Read joystick inputs in x and y, cleaning up and limiting where necessary
int readInput() {
  int joystick_x_in = analogRead(joyXPin);
  int joystick_y_in = analogRead(joyYPin);
  //Serial.print("Read inputs ");
  Serial.println(joystick_y_in);

  // Note: Changed to normal Arduino map function because betterMap is somehow wrong (I didn't do the math to figure out why). Normal map function seems to work well though. --Ryan Meshulam
  joystick_x_in = map(joystick_x_in, 101, 922, 0, 180);
  joystick_y_in = map(joystick_y_in, 101, 923, 0, 180);
  //Serial.print(joystick_x_in);
  if (joystick_x_in < 97 && joystick_x_in > 83) {
    joystick_x_in = 90;
  }
  if (joystick_y_in < 97 && joystick_y_in > 83) {
    joystick_y_in = 90;
  }

  return(joystick_x_in, joystick_y_in);
}

//Define a limited range of motion for servos so the servos can't turn the go beyond the stall (we think)
int servoLimiter(int inputDegrees){
  if (inputDegrees>90+SERVORANGE){
    return 90+SERVORANGE;
  } else if (inputDegrees<90-SERVORANGE){
    return 90-SERVORANGE;
  } else {
    return inputDegrees;
  }
}
