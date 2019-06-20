// Elec steering and buoyancy code 2018
// SUBC: the UBC Submarine Design Team

// Libraries for communicating with IMU
#include <Wire.h>
#include "I2Cdev.h"
#include <MPU6050.h>
// Steering libraries
#include <Servo.h>
// Buoyancy libraries
#include <MS5837.h>
// Display library (Adafruit Neopixel Jewel)
#include <Adafruit_NeoPixel.h>

// Initialize servos
Servo xservo;
Servo yservo;

// Display constants
#define JEWEL_PIN 7
#define NEOPIXEL_COUNT 7

// Steering constants
const int joyXPin = A0;
const int joyYPin = A1;
const int xservoPin = 9;
const int yservoPin = 10;
const int manualSwitchPin = 11;
#define MANUAL true
#define AUTOMATED false

// Smoothing constants
#define SMOOTHINGERROR 2
#define ORIGINERROR 10

#define SERVORANGE 36

// Manual variables and objects
int joystick_y_in, joystick_x_in;
bool manualSwitch = MANUAL;

// IMU variables and objects
MPU6050 sensor;
bool sensorWorking = false;
int16_t ax, ay, az, oldAX=90;
int16_t gx, gy, gz;

// Buoyancy sensor
MS5837 depthSensor;

// Display object and helpers
Adafruit_NeoPixel jewel(NEOPIXEL_COUNT, JEWEL_PIN, NEO_GRB + NEO_KHZ800);
uint32_t red, blue, green, purple;

int servoLimiter(int);

void setup(){

  // Initialize serial monitoring
  Serial.begin(9600);
  
  // Initialize servos
  yservo.attach(yservoPin);
  xservo.attach(xservoPin);

  // Initialize manual control
  pinMode(manualSwitchPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(manualSwitchPin), switchManual, CHANGE);

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

  // Initialize display
  jewel.begin();
  jewel.fill(jewel.Color(0,0,0));
  jewel.setBrightness(20);
  jewel.show();
  red=jewel.Color(255,0,0);
  blue=jewel.Color(0,0,255);
  green=jewel.Color(0,205,14);
  purple=jewel.Color(212, 17, 242);

  // Initialize buoyancy sensor
  if (!depthSensor.init()){
    Serial.println("Buoyancy sensor initialization failed!");
    jewel.fill(red);
    delay(1000);
    jewel.fill(jewel.Color(0,0,0));
  }
  depthSensor.setModel(MS5837::MS5837_02BA);
  depthSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}


void loop(){
  switchManual();
  if (manualSwitch==MANUAL){
    Serial.println("MANUAL");
    jewel.fill(green);
  } else {
    if (sensor.testConnection()!=true){
      sensor.initialize();
    }
    Serial.println("AUTOMATED");
    jewel.fill(purple);
  }
  jewel.show();
  // Since x is not controlled by the IMU, we can put it outside of the for loop.
  readInput();
  joystick_x_in=servoLimiter(joystick_x_in);
  xservo.write(joystick_x_in);
  Serial.print("X:");
  Serial.print(joystick_x_in);
  
  if (manualSwitch==AUTOMATED) {
    sensor.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);
    /*Serial.print("IMU:");
    Serial.print(ax);
    Serial.print(" ");
    Serial.print(gx);
    Serial.print(" ");
    Serial.print(ay, 4);
    Serial.print(gy,4);
    Serial.print(" ");
    Serial.print(az, 4);
    Serial.print(gz,4);*/
    ax = map (ax, -17000, 21000, 0, 180) ;

    if (ax>90-ORIGINERROR&&ax<90+ORIGINERROR){
      ax=90;
    } else if(ax>oldAX-SMOOTHINGERROR&&ax<oldAX+SMOOTHINGERROR){
      ax=oldAX;
    }

    ax=servoLimiter(ax);
    Serial.print(" AutomatedY:");
    Serial.print(ax);

    Serial.println("");

    yservo.write (ax);
    oldAX=ax;
    //delay (200);
  } else {
    //Serial.println("Manual steering is on!");
    joystick_y_in=servoLimiter(joystick_y_in);
    yservo.write(joystick_y_in);

    Serial.print(" ManualY:");
    Serial.println(joystick_y_in);
  }

  // Buoyancy printout
  depthSensor.read();
  Serial.print("Pressure: "); 
  Serial.print(depthSensor.pressure()); 
  Serial.println(" mbar");
  
  Serial.print("Temperature: "); 
  Serial.print(depthSensor.temperature()); 
  Serial.println(" deg C");
  
  Serial.print("Depth: "); 
  Serial.print(depthSensor.depth()); 
  Serial.println(" m");
  
  Serial.print("Altitude: "); 
  Serial.print(depthSensor.altitude()); 
  Serial.println(" m above mean sea level");
  delay(10);

}

void readInput() {
  joystick_x_in = analogRead(joyXPin);
  joystick_y_in = analogRead(joyYPin);
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
}

void switchManual(){
  manualSwitch = digitalRead(manualSwitchPin);
  Serial.print("result of digitalread");
  Serial.println(manualSwitch);
  sensorWorking = sensor.testConnection();
  Serial.println(sensorWorking);
  if (sensorWorking){
    manualSwitch = digitalRead(manualSwitchPin);
  } else {
    manualSwitch = MANUAL;
  }
  manualSwitch=AUTOMATED;
}

int servoLimiter(int inputDegrees){
  if (inputDegrees>90+SERVORANGE){
    return 90+SERVORANGE;
  } else if (inputDegrees<90-SERVORANGE){
    return 90-SERVORANGE;
  } else {
    return inputDegrees;
  }
}
