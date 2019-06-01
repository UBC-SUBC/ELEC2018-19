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
