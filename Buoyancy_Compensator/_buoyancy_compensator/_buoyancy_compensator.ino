/*
*  buoyancy compensator code 
*  Last undated June 23, 2019 by Yuki Sekimori
* 
* 
*  Things to add next:
*  3. Control algorithm
*  4. No more encoder :(
*  5. KF if I can do it haha
*  6. limit switch 
*/

/*Libraries*/
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h> //IMU library
#include <MS5837.h> //pressure senror library

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
int mode = FW;

// motor PWM 
// Ranging from (0 - 255)
#define pwm 50
//int pwm;


/******Arudino pin definitions******/
// Pins connected to Pololu Motor Driver
#define dirPin 2 // motor direction output pin
#define pwmPin 3 // pulse width modulation output pin
#define slpPin 4 // sleep mode output pin. default LOW. 
                  // Must be set to HIGH to enable the driver
// REMOVED FOR SIMPLICITY
//#define fltPin 10 // fault indicator input pin. 
                  // This is driven LOW when fault has occurred.
//#define csPin A1 // current sense output (analog input pin)

// Pins connected to the motor hall sensors (input pin)
//#define hs1Pin 7
//#define hs2Pin 8
// Pins connected to the limit switches (input pin)
#define ls1Pin 12
#define ls2Pin 13

// IMU variables and objects
MPU6050 IMU;
bool IMUWorking = false;
int16_t ax, ay, az, azPrev, oldAX=90;
int16_t gx, gy, gz;

// Buoyancy sensor (bar_02) setup
MS5837 bar_02;
double setDepth;
double depth;
bool endBuoyancy;

// Limit switch setup
int lsCount;

void setup()
{
  //setup serial communication
  Serial.begin(9600);
  Serial.println("Starting");

  Wire.begin();
  // Initialize IMU
  Wire.begin();
  Serial.println( "Initializing the Sensors" );
  IMU.initialize ( );
  IMUWorking = IMU.testConnection();
  if (IMUWorking == true){
    Serial.println ("IMU Successfully Connected");
    delay (1000);
    Serial.println ( "Taking Values from the IMU sensor" );
    delay (1000);
  } else {
    Serial.println("Connection failed");
  }
  
  // initialize bar02(MS5837) pressure sensor
  bar_02.setModel(MS5837::MS5837_02BA);
  if (!bar_02.init()){
    Serial.println("Pressure sensor initialization failed!");
    delay(1000);
  }
  bar_02.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
  bar_02.read();//read pressure sensor
  setDepth = bar_02.depth(); //get the set depth value
  endBuoyancy = LOW;
  
  
  // setup digital pin modes
  pinMode(dirPin, OUTPUT);
  pinMode(slpPin, OUTPUT);
  //Motor encoder is not working anymore :(
  //pinMode(hs1Pin, INPUT);
  //pinMode(hs2Pin, INPUT);
  pinMode(ls1Pin, INPUT);
  pinMode(ls2Pin, INPUT);
  
}

/**********MAIN FUNCTION**********/
void loop()
{
  while (endBuoyancy == LOW){
    //readSensors();//input singals. For troubleshooting purpose.\

    //buoyancy control algorithm
    bar_02.read();//read pressure sensor
    depth = bar_02.depth(); //get depth value
    IMU.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz); //get IMU values
    if (abs(az) < 1000){ //stop the motor if the z accelearation is within the threshold
      mode = BR; //stop the motor
      endBuoyancy = HIGH;
    }
    else if (abs(setDepth - bar_02.depth()) < 0.1){//move the piston until the sub is close to the set depth
      if (abs(az - azPrev) > 0 ){
        mode = abs(mode - 1); //change the motor direction
      }
      //renew the setDepth
      setDepth = bar_02.depth();
    }
    
    //check the limit sensor
    if (digitalRead(ls1Pin) == LOW && digitalRead(ls2Pin) == LOW) {
      lsCount = 0; //reset the ls Count
    }
    else if (digitalRead(ls1Pin) == HIGH && digitalRead(ls2Pin) == HIGH) {
      mode = BR; //stop the motor
    }
    else {
      if (lsCount == 0 ){
        mode = abs(mode - 1); //change the motor direction
      }
      else if (lsCount > 0 && lsCount < 10) { //change the number based on experiment
        
      }
      else {
      mode = BR; //stop the motor
      endBuoyancy = HIGH; 
      break;
      }
    }
    lsCount++;
      
    motorDriver(mode,pwm);
    }

}

/********FUNCTIONS********/

/***************MOTOR DRIVER****************/
void motorDriver(int motorMode, int motorPwm){
  // Check if the motor enable is assingned properly
  if (motorEnable == HIGH || motorEnable == LOW){
    
    // Check if the motor mode is assigned correctly
    if (motorMode == FW || motorMode == BW){

      // Check if the pwm value is within a correct range
      if (motorPwm >= 0 || motorPwm <= 255){ 
        digitalWrite(dirPin, motorMode); // assign the mode of the motor
        digitalWrite(slpPin, motorEnable); // assign the slp pin of the driver
        analogWrite(pwmPin, motorPwm); // assign the pwm value of the motor
        Serial.print("MODE:");
        Serial.print(motorMode); // motor mode
        Serial.print("\t");
        Serial.print("PWM:");
        Serial.print(motorPwm); // motor PWM
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
void readSensors() {
  // Read the input values
  Serial.print("READ SENSOR:\t");
  //Serial.print("FLT:");
  //Serial.print(digitalRead(fltPin)); // FLT pin readings
  Serial.print("\t");
  //Serial.print("HS1:");
  //Serial.print(digitalRead(hs1Pin)); // hall sensor 1 readings
  //Serial.print("\t");
  //Serial.print("HS2:");
  //Serial.print(digitalRead(hs2Pin)); // hall sensor 2 readings
  //Serial.print("\t");
  Serial.print("LS1:");
  Serial.print(digitalRead(ls1Pin)); // limit switch 1 readings
  Serial.print("\t");
  Serial.print("LS2:");
  Serial.print(digitalRead(ls2Pin)); // limit switch 2 readings
  Serial.print("\t");
  bar_02.read();
  Serial.print("Pressure: "); 
  Serial.print(bar_02.pressure()); 
  Serial.print(" mbar");
  Serial.print("\t");
  Serial.print("Temperature: "); 
  Serial.print(bar_02.temperature()); 
  Serial.print(" deg C");
  Serial.print("\t");
  Serial.print("Depth: "); 
  Serial.print(bar_02.depth()); 
  Serial.print(" m");
  Serial.print("\t");
  Serial.print("Altitude: "); 
  Serial.print(bar_02.altitude()); 
  Serial.println(" m above mean sea level");
}
