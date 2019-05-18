//
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
