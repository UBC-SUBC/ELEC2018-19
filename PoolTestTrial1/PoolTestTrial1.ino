/*
// Reading current values to the SD card 
// Reference Code:
// https://www.arduino.cc/en/Tutorial/ReadWrite
// ELEC steering and Buoyancy Compensator
*/
/* Things to add later
 * Serial read so that it can change the setup from the serial command.
 */

/*****SD CARD WRITING SETUP*****/

#include <SPI.h>
#include <SD.h>

File PoolTest;
int loopCount;
/*****SD CARD WRITING SETUP OVER*****/

/*****ELEC STEERING START*****/

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

/****ELEC STEERING END****/

/*****BUOYANCY INTRO*****/

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
//#define fltPin 10 // fault indicator input pin. 
                  // This is driven LOW when fault has occurred.
//#define csPin A1 // current sense output (analog input pin)
// Pins connected to the motor hall sensors (input pin)
#define hs1Pin 7
#define hs2Pin 8
// Pins connected to the limit switches (input pin)
#define ls1Pin 12
#define ls2Pin 13
// Pins connected to the pressure sensor (analog input pin)
#define psPin A2

int mode;
/*****BUOYANCY INTRO DONE*****/


void setup()
{
 /*****SD Card to Serial Port setup START****/ 
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  loopCount = 0; //set loop count to 0
 /*****SD Card to Serial Port setup END****/ 
 
  //buoyancy compensator setup
  pinMode(dirPin, OUTPUT);
  pinMode(slpPin, OUTPUT);
  //pinMode(fltPin, INPUT);
  pinMode(hs1Pin, INPUT);
  pinMode(hs2Pin, INPUT);
  pinMode(ls1Pin, INPUT);
  pinMode(ls2Pin, INPUT);
  //pinMode(csPin, INPUT);
  pinMode(psPin, INPUT);

  //elec steering servo setup 
  delay(200);
  xservo.attach(xservoPin);
  yservo.attach(yservoPin);
  pinMode(manualSwitchPin, INPUT_PULLUP);
 
}

/*************MAIN LOOP***************/
void loop() {
/******CREATING FILES IN SD CARD STARTS HERE*******/
  if (loopCount > 999){
    // close the file after 1000 loops before making a new one
    PoolTest.close();
    Serial.println("file closed.");
    loopCount = 0;
  }
  if (loopCount == 0){
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    char filename[] = "DATA000.CSV";
    for (uint8_t i = 0; i < 500; i++) {
        filename[4] = i/100 + '0';
        filename[5] = i%100 + '0';
        filename[6] = i%10 + '0';
        if (! SD.exists(filename)) {
          // only open a new file if it doesn't exist
          PoolTest = SD.open(filename, FILE_WRITE);
          break;  // leave the loop!
        }
    }
    // if the file opened okay, write to it:
    if (PoolTest) {
    Serial.print("Writing to a new file");
    PoolTest.println("apple, banana, cabbage"); //header line
    } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
    }  
  }
  loopCount++;
/******CREATING FILES IN SD CARD ENDS HERE*******/

  /*****ELEC STEERING STARTS HERE*******/
  manualSwitch = digitalRead(manualSwitchPin);
  if (manualSwitch == 1){
    Serial.println("Manual steering is on!");
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
  /*****ELEC STEERING ENDS HERE*******/
  
 
 /**********BUOYANCY MAIN FUNCTION STARTS HERE**********/
 // readSensor();
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
  /**********BUOYANCY MAIN FUNCTION ENDS HERE**********/
  
}


         /********FUNCTIONS********/
/********BUOYANCY COMPENSATOR FUNCTIONS******/
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
  //Serial.print(digitalRead(fltPin)); // FLT pin readings
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
  //Serial.print("CS:");
  //Serial.print(currentSensor(analogRead(csPin))); // current sensor readings
  //Serial.print("[A]");
  Serial.print("\n");
}

/****CURRENT SENSOR CONVERSION*****/
double currentSensor(int cs){
  double i;
  i = (cs * 5 / 1024 -0.05)/ 0.02; //A
  return i;
}

/*****ELEC STEERING FUNCTIONS*****/

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
