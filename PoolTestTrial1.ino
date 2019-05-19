// Buoyancy Compensator

// MC A1 top limit switch (W)
// MC A2 bottom limit switch (G)
//
//

#include <SoftwareSerial.h>
const byte interruptPin = 2;
#define encoderPin 3 //Enconder
volatile int position = 0;
#define rxPin 4    // pin 4 connects to SMC TX
#define txPin 5    // pin 5 connects to SMC RX
#define resetPin 6 // pin 6 connects to SMC nRST
#define errPin 7   // pin 7 connects to SMC ERR
#define controlPin A3
SoftwareSerial smcSerial = SoftwareSerial(rxPin, txPin);
 
// some variable IDs
#define ERROR_STATUS 0
#define LIMIT_STATUS 3
#define TARGET_SPEED 20
#define INPUT_VOLTAGE 23
#define TEMPERATURE 24
#define MAXPOS 12000
#define MINPOS 1000
#define MAXSPEED 3200
 
// some motor limit IDs
#define FORWARD_ACCELERATION 10
#define REVERSE_ACCELERATION 10
#define DECELERATION 10
#define SAMPLEPERIOD 10
#define P_TO_V 14

  double KP = 1;
  int input_[SAMPLEPERIOD];
  long input_t;
  int pos_[SAMPLEPERIOD];
  long pos_t;
  double motor_speed = 0;
  int time_ = 0;
// read a serial byte (returns -1 if nothing received after the timeout expires)
int readByte()
{
  char c;
  if(smcSerial.readBytes(&c, 1) == 0){ return -1; }
  return (byte)c;
}
 
// required to allow motors to move
// must be called when controller restarts and after any error
void exitSafeStart()
{
  smcSerial.write(0x83);
}
 
// speed should be a number from -3200 to 3200
void setMotorSpeed(int speed)
{
  if (speed < 0)
  {
    smcSerial.write(0x86);  // motor reverse command
    speed = -speed;  // make speed positive
  }
  else
  {
    smcSerial.write(0x85);  // motor forward command
  }
  smcSerial.write(speed & 0x1F);
  smcSerial.write(speed >> 5);
}
 
unsigned char setMotorLimit(unsigned char  limitID, unsigned int limitValue)
{
  smcSerial.write(0xA2);
  smcSerial.write(limitID);
  smcSerial.write(limitValue & 0x7F);
  smcSerial.write(limitValue >> 7);
  return readByte();
}
 
// returns the specified variable as an unsigned integer.
// if the requested variable is signed, the value returned by this function
// should be typecast as an int.
unsigned int getVariable(unsigned char variableID)
{
  smcSerial.write(0xA1);
  smcSerial.write(variableID);
  return readByte() + 256 * readByte();
}


void Count() 
{
  if(digitalRead(encoderPin) == HIGH){
    position++;
  }
  else{
    position--;
  }
}

void Zero(){
  setMotorSpeed(-1000);  // full-speed backwards
  int reached_end = 0;
  while(reached_end != 0x10)
  {
    if (digitalRead(errPin) == HIGH)
    {
      reached_end = (getVariable(ERROR_STATUS), HEX);
      exitSafeStart();
    }
  }
  exitSafeStart();
  setMotorSpeed(0);
  delay(100);
  position = 0;
}
 
void setup()
{

  Serial.begin(115200);    // for debugging (optional)
  smcSerial.begin(9600);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), Count, RISING);

  
  // briefly reset SMC when Arduino starts up (optional)
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);  // reset SMC
  delay(1);  // wait 1 ms
  pinMode(resetPin, INPUT);  // let SMC run again
 
  // must wait at least 1 ms after reset before transmitting
  delay(5);
 
  // this lets us read the state of the SMC ERR pin (optional)
  pinMode(errPin, INPUT);
 
  smcSerial.write(0xAA);  // send baud-indicator byte
  setMotorLimit(FORWARD_ACCELERATION, 15);
  setMotorLimit(REVERSE_ACCELERATION, 15);
  setMotorLimit(DECELERATION, 5);
  // clear the safe-start violation and let the motor run
  exitSafeStart();

  Zero();
}
 
void loop()
{
  input_[time_] = analogRead(controlPin);
  pos_[time_] = position;
  Serial.println(position);
      //int pr = (int)(analogRead(controlPin)/4);

        //Serial.write(pr);
  
  delay(10);
  
  if(time_ == 0)
    {
      input_t = 0;
      pos_t = 0;
      for (int i = 0; i<SAMPLEPERIOD; i++)
      {
        input_t += input_[i];
        pos_t += pos_[i];
      }
      motor_speed = KP*(input_t*P_TO_V-pos_t)/SAMPLEPERIOD;
      if (pos_t/SAMPLEPERIOD > MAXPOS || pos_t/SAMPLEPERIOD < MINPOS)
      motor_speed /= 5;
      if(motor_speed>MAXSPEED)
        motor_speed = MAXSPEED; 
      else if(motor_speed<-MAXSPEED)
        motor_speed = -MAXSPEED; 
      setMotorSpeed(motor_speed); 
   

    }



//      Serial.print(255);
      
//      int pr = (int)(analogRead(controlPin)/4);
//
//        Serial.write(pr);
//      if( pr / 256 >= 255){
//        Serial.print(254);
//      }else{
//        Serial.print(pr/256);
//      }

//      int sp = (int)motor_speed ;
//      if( sp % 256 >= 255){
//        Serial.print(254);
//      }else{
//        Serial.print(sp%256);
//      }
//      
//      if( sp / 256 >= 255){
//        Serial.print(254);
//      }else{
//        Serial.print(sp / 256);
//      }
//
//      int po  = position;
//      if((po % 256) >= 255){
//        Serial.print(254);
//      }else{
//        Serial.print(po % 256);
//      }
//      
//      if((po / 256) >= 255){
//        Serial.println(254);
//      }else{
//        Serial.println(po / 256);
//      }

    time_ = ++time_ % SAMPLEPERIOD;
 
  // if an error is stopping the motor, write the error status variable
  // and try to re-enable the motor
  if (digitalRead(errPin) == HIGH)
  {
//    Serial.print("Error Status: 0x");
//    Serial.println(getVariable(ERROR_STATUS), HEX);
    // once all other errors have been fixed,
    // this lets the motors run again
    exitSafeStart();
  }
}

//Elec Steering joystick servo

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
