// Buoyancy Compensator

/*
// Simple buoyancy compensator code
// 
*/
/* Things to add later
 * Serial read so that it can change the setup from the serial command.
 */


/************MOTOR SETUP***********/
// motor on(HIGH)/off(LOW)
#define motorEnable HIGH
// motor mode setup
#define FW 0
#define BW 1
#define BR 2
/***** Motor Driver Truth Table*****
* MODE PWM DIR OUTA OUTB Operation *
* FW=1  H   H   H    L    Forward  *
* BW=0  H   L   L    H    Reverse  *
* BR=2  L   X   L    L    HardBrake*
***********************************/
//#define motorMode BW

// motor PWM setup
// Ranging from (0 - 255)
#define motorPWM 50


/******Arudino pin definitions******/
// Pins connected to Pololu Motor Driver
#define dirPin 2 // motor direction output pin
#define pwmPin 3 // pulse width modulation output pin
#define slpPin 4 // sleep mode output pin. default LOW. 
                  // Must be set to HIGH to enable the driver
#define fltPin 10 // fault indicator input pin. 
                  // This is driven LOW when fault has occurred.
#define csPin A1 // current sense output (analog input pin)
// Pins connected to the motor hall sensors (input pin)
#define hs1Pin 7
#define hs2Pin 8
// Pins connected to the limit switches (input pin)
#define ls1Pin 12
#define ls2Pin 13
// Pins connected to the pressure sensor (analog input pin)
#define psPin A2

int mode;

void setup()
{
  Serial.begin(9600);
  // setup digital pin modes
  pinMode(dirPin, OUTPUT);
  pinMode(slpPin, OUTPUT);
  pinMode(fltPin, INPUT);
  pinMode(hs1Pin, INPUT);
  pinMode(hs2Pin, INPUT);
  pinMode(ls1Pin, INPUT);
  pinMode(ls2Pin, INPUT);
  pinMode(csPin, INPUT);
  pinMode(psPin, INPUT);
  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));
}

/**********MAIN FUNCTION**********/
void loop()
{
  readSensor();
  if (digitalRead(ls1Pin) == 0 && digitalRead(ls2Pin) == 0) {
    
  }
  else if (digitalRead(ls1Pin) == 1 && digitalRead(ls2Pin) == 0) {
    mode = 0;
  }
  else if (digitalRead(ls1Pin) == 0 && digitalRead(ls2Pin) == 1) {
    mode = 1;
  }
  else {
    mode = 2;
  }
  motorDriver(mode);
}

/********FUNCTIONS********/

/***************MOTOR DRIVER****************/
void motorDriver(int motorMode){
  // Check if the motor enable is assingned properly
  if (motorEnable == HIGH || motorEnable == LOW){
    
    // Check if the motor mode is assigned correctly
    if (motorMode == FW || motorMode == BW){

      // Check if the pwm value is within a correct range
      if (motorPWM >= 0 || motorPWM <= 255){ 
        digitalWrite(dirPin, motorMode); // assign the mode of the motor
        digitalWrite(slpPin, motorEnable); // assign the slp pin of the driver
        analogWrite(pwmPin, motorPWM); // assign the pwm value of the motor
        Serial.print("MODE:");
        Serial.print(motorMode); // motor mode
        Serial.print("\t");
        Serial.print("PWM:");
        Serial.print(motorPWM); // motor PWM
        Serial.print("\t");
      }
      
      else {
        digitalWrite(slpPin, LOW); // turn off the motor driver
        Serial.println("Error: motor pwm value not assingned in a correct range");
      }
    }
    else if(motorMode == BR){
      analogWrite(pwmPin, LOW); // assign LOW to pwmPin to break
      Serial.print("MODE:2\t"); // motor mode (BR = 2)
      Serial.print("PWM:0\t"); // motor mode (LOW = 0)
    }
    
    else {
      digitalWrite(slpPin, LOW); // turn off the motor driver
      Serial.println("Error: motor mode not assigned correctly.");
    }
  }

  else {
    digitalWrite(slpPin, LOW); // turn off the motor driver
    Serial.println("Error: motor driver not enabled correctly");
  }
}

/**********READING SENSOR INPUTS**************/
void readSensor() {
  // Read the input values
  Serial.print("FLT:");
  Serial.print(digitalRead(fltPin)); // FLT pin readings
  Serial.print("\t");
  Serial.print("HS1:");
  Serial.print(digitalRead(hs1Pin)); // hall sensor 1 readings
  Serial.print("\t");
  Serial.print("HS2:");
  Serial.print(digitalRead(hs2Pin)); // hall sensor 2 readings
  Serial.print("\t");
  Serial.print("LS1:");
  Serial.print(digitalRead(ls1Pin)); // limit switch 1 readings
  Serial.print("\t");
  Serial.print("LS2:");
  Serial.print(digitalRead(ls2Pin)); // limit switch 2 readings
  Serial.print("\t");
  Serial.print("PS:");
  Serial.print(analogRead(psPin)); // pressure sensor readings
  Serial.print("\t");
  Serial.print("CS:");
  Serial.print(currentSensor(analogRead(csPin))); // current sensor readings
  Serial.print("[A]");
  Serial.print("\n");
}

/****CURRENT SENSOR CONVERSION*****/
double currentSensor(int cs){
  double i;
  i = (cs * 5 / 1024 -0.05)/ 0.02; //A
  return i;
}

//
//Elec Steering joystick servo
//

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


//SD card writing autoBollard Code

// Auto-Bollard 2018
// GPL v3
// SUBC: the UBC Submarine Design Team

//#include "HX711.h"
#include <SD.h>
#include <SPI.h>

// Load cell calibration
#define calibration_factor -10860.0 //calibrated with 750g test load

// Change when plugged in to Arduino
#define tachoPin 2
#define BUTTONPIN 3
#define INDICATORPIN 9
#define DOUT  5
#define CLK  6

//#define BEFORECALC 4
#define MEASUREDELAY 500
#define CHANGEPERREV 2
#define DELAYTIME 5
#define DEBOUNCETIME 250

// Load cell object
HX711 scale;

volatile int changes=0;
unsigned long startTime;
unsigned long endTime;
unsigned long recordTime;
int duration;
int oldFall=0;
boolean running=false;
//boolean oldRunning=false;
boolean initialized=false;

/*// Debounce stuff
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers*/

File outputFile;

void buttonPress(void);
void tachoChange(void);

void setup() {
  Serial.begin(9600);
  while (!Serial){
    // Wait for Serial to connect
  }
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // Button stuff
  pinMode(BUTTONPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), buttonPress, FALLING);
  attachInterrupt(digitalPinToInterrupt(tachoPin), tachoChange, CHANGE);

  pinMode(INDICATORPIN, OUTPUT);
  digitalWrite(INDICATORPIN, LOW);

  // Load cell initialization
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor); 
  scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0
  
}

void loop() {
  if(running==true&&initialized==true&&millis()-startTime>MEASUREDELAY){
    endTime=millis();
    double RPM = (60000.0*changes/CHANGEPERREV)/(endTime-startTime);
  
    Serial.print("Time: ");
    duration=(endTime-recordTime);
    outputFile.print(duration);
    Serial.print(duration);
  
    outputFile.print(",");
  
    Serial.print("    RPM: ");
    Serial.print(RPM);
    
    /*Serial.print("   changes: ");
    Serial.println(changes);*/
    
    outputFile.print(RPM);

    Serial.print(scale.get_units(), 2); //"scale.get_units(), n" returns a float with n sigfigs
    Serial.println(" N"); //Units can change but you'll need to refactor the calibration_factor

    outputFile.print(",");
    outputFile.println(scale.get_units());

    changes = 0;
    startTime=millis();
  } else {
    delay(DELAYTIME);
  }
}
  

void buttonPress(void){
  Serial.println("Button falling!");
  if (millis()-oldFall>DEBOUNCETIME){
    oldFall=millis();
    if (running==true&&initialized==true){
      // Stuff to close file, etc.
      outputFile.close();
      Serial.println("Closing file...");
      initialized=false;
      running=false;
      digitalWrite(INDICATORPIN, LOW);
      delay(100);
    } else if (running==false&&initialized==false){
      // Stuff to open file, etc.
      // create a new file
      char filename[] = "DATA00.CSV";
      for (uint8_t i = 0; i < 100; i++) {
        filename[4] = i/10 + '0';
        filename[5] = i%10 + '0';
        if (! SD.exists(filename)) {
          // only open a new file if it doesn't exist
          outputFile = SD.open(filename, FILE_WRITE);
          break;  // leave the loop!
        }
      }
      outputFile.println("Time(ms),RPM,Force(N)");
      //outputFile.println("RPM");
      Serial.print(filename);
      Serial.println(" successfully initialized!");
      digitalWrite(INDICATORPIN, HIGH);
      initialized=true;
      running=true;
  
      changes=0;
      startTime=millis();
      recordTime=startTime;
    } else {
      Serial.println("Something went wrong. Trying to close file.");
      outputFile.close();
    }
  } else {
    Serial.println("Not enough time has passed since the last event. Debouncing.");
  }
}

void tachoChange(void) {
  changes++;
}
