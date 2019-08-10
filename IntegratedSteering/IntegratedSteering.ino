// Elec steering and buoyancy code 2018
// SUBC: the UBC Submarine Design Team

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

// ANNA? 
// Steering constants
#define MANUAL true
#define AUTOMATED false

// Smoothing constants
#define SMOOTHINGERROR 2
#define ORIGINERROR 10

#define SERVORANGE 36

// ANNA??
// Manual variables and objects
int joystick_y_in, joystick_x_in;
bool manualSwitch = MANUAL;

// IMU variables and objects

MPU6050 IMU;
bool IMUWorking = false;
int16_t ax, ay, az, azPrev, oldAX=90;
int16_t gx, gy, gz;

// Pin A4 and A5 are reserved for I2C sensors

// Initialize servos
Servo xservo;
Servo yservo;

/** No buoyancy compensator so these are unnecessary
  //bar_02 pressure sensor libraries
  #include <MS5837.h>
  Display library (Adafruit Neopixel Jewel)
  #include <Adafruit_NeoPixel.h> 
 */

/***** All of these are unnecessary because no buoyancy compensator yet
  // Switch attached currently, moving to a software switch
    // const int manualSwitchPin = 11;

    // Pins connected to Pololu Motor Driver
    //#define dirPin 2 // motor direction output pin
    //#define pwmPin 3 // pulse width modulation output pin
    //#define slpPin 4 // sleep mode output pin. default LOW. 
                  // Must be set to HIGH to enable the driver
    // Pins connected to the limit switches (input pin)
    //#define ls1Pin 12
    //#define ls2Pin 13
******/

/*** Start No display attached currently 
  // Display constants
  //#define JEWEL_PIN 7
  //#define NEOPIXEL_COUNT 7

End No display currently attached ***/

/*** START No Buoyancy Compensator Attached 
  // Buoyancy compensator setup

  bool endBuoyancy;
  **********bar_02 SETUP***********
  
  MS5837 bar_02; // bar_02 pressure sensor
  double setDepth;
  double depth;
  ************MOTOR SETUP***********
  
  motor on(HIGH)/off(LOW)
  #define motorEnable HIGH
  motor mode setup
  #define FW 0
  #define BW 1
  #define BR 2
  /***** Motor Driver Truth Table*****
  * MODE PWM DIR OUTA OUTB Operation *
  * FW=1  H   H   H    L    Forward  *
  * BW=0  H   L   L    H    Reverse  *
  * BR=2  L   X   L    L    HardBrake*
  ***********************************
  #define motorMode BW
  int mode = FW;
  motor PWM: Ranging from (0 - 255)
  #define pwm 100
  int pwm;
  int  initPWM = 100; //initial motor output (adjustable)
  /************az SETUP***********
  const int azThreshold = 1000; //threshold for buoyancy accelearation (adjustable)
  /************LIMIT SWITCH SETUP***********
  // int lsCount; // Limit switch setup
  // End of buoyancy compensator setup

  // Display object and helpers
  // Adafruit_NeoPixel jewel(NEOPIXEL_COUNT, JEWEL_PIN, NEO_GRB + NEO_KHZ800);
  // uint32_t red, blue, green, purple;

  int servoLimiter(int);

End No Buoyancy Compensator Attached 
*****/ 


/******
***
MAIN SETUP FUNCTION
***
******/

void setup(){

  // Initialize serial monitoring
  Serial.begin(9600);
  
  // Initialize servos
  yservo.attach(yservoPin);
  xservo.attach(xservoPin);

  // Initialize manual control
  // pinMode(manualSwitchPin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(manualSwitchPin), switchManual, CHANGE);

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
  // switchManual();

  /*** START No display currently attached
    // Initialize display and variables
      jewel.begin();
      jewel.fill(jewel.Color(0,0,0));
      jewel.setBrightness(20);
      jewel.show();
      red=jewel.Color(255,0,0);
      blue=jewel.Color(0,0,255);
      green=jewel.Color(0,205,14);
      purple=jewel.Color(212, 17, 242);

      Initialize bar02(MS5837) pressure sensor
        bar_02.setModel(MS5837::MS5837_02BA);
        if (!bar_02.init()){
        Serial.println("Pressure sensor initialization failed!");
        delay(1000);
      }
      
      bar_02.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
      bar_02.read();//read pressure sensor
      // setDepth = bar_02.depth(); //get the set depth value
    // Initialize buoyancy compensator variable
    // endBuoyancy = LOW;
    // pwm = initPWM;
  } 
 
END No display currently attached***/
}


/********MAIN LOOP FUNCTION********/

void loop(){

  // jewel.show();
  
// Since x is not controlled by the IMU, we can put it outside of the for loop.
  readInput();
  joystick_x_in=servoLimiter(joystick_x_in);
  xservo.write(joystick_x_in);
  Serial.print("X:");
  Serial.print(joystick_x_in);
  
  if (manualSwitch==AUTOMATED) {
    IMU.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);
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
    } 
    
    else if(ax>oldAX-SMOOTHINGERROR&&ax<oldAX+SMOOTHINGERROR){
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

/******BUOYANCY COMPENSATOR START*******/
    /** while (endBuoyancy == LOW){
      readSensors();//input singals. For troubleshooting purpose. **\

/********* NO BUOYANCY COMPENSATOR
    //buoyancy control algorithm
      bar_02.read();//read pressure sensor
      depth = bar_02.depth(); //get depth value
      IMU.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz); //get IMU values
      if (abs(az) < azThreshold){
      if (abs(setDepth - bar_02.depth()) < 0.1){
        //stop the motor if the z accelearation is within the threshold
        //and the submarine is at close to the set depth
        mode = BR; //stop the motor
        endBuoyancy = HIGH;
        break;
      }
      //slow down the motor when the motor is in the acceleration threshold
      pwm--;
    }
    else if ( (abs(az) - abs(azPrev))  > 0 ){
      //compare the current z acceleration with the previous value
      //and change dirction if the current value is accelerating more 
      //than the previous value
      mode = abs(mode - 1); //change the motor direction      
    }
    //renew the setDepth and azPrev
    setDepth = bar_02.depth();
    azPrev = az;
        
    //check the limit sensor
    if (digitalRead(ls1Pin) == LOW && digitalRead(ls2Pin) == LOW) {
      lsCount = 0; //reset the lsCount
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
    BUOYANCY COMPENSATOR END*******/
    
/****  switchManual();
  if (manualSwitch==MANUAL){
    Serial.println("MANUAL");
    // jewel.fill(green);
  } else {
    if (IMU.testConnection()!=true){
      IMU.initialize();
    }
    Serial.println("AUTOMATED");
    // jewel.fill(purple);
  }
 ****/
  
}


/********FUNCTIONS********/
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

// ANNA? 
/** Reformat this to make it a code switch instead of a manual switch
void switchManual(){
    manualSwitch = digitalRead(manualSwitchPin);
    Serial.print("result of digitalread");
    Serial.println(manualSwitch);
    IMUWorking = IMU.testConnection();
    Serial.println(IMUWorking);
    if (IMUWorking){
      manualSwitch = digitalRead(manualSwitchPin);
    } else {
      manualSwitch = MANUAL;
    }
    manualSwitch=AUTOMATED;
  }
***/


/*** Start No Motor Driver because no buoyancy compensator 

  ***************MOTOR DRIVER****************
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
            Serial.println("Error: motor pwm value not assingned in a correct range. End buoyancy compensator...");
            endBuoyancy = HIGH;
          }
        }
      
        else if(motorMode == BR){
          analogWrite(pwmPin, LOW); // assign LOW to pwmPin to break
          Serial.print("MODE:2\t"); // motor mode (BR = 2)
          Serial.print("PWM:0\t"); // motor mode (LOW = 0)
        }
    
        else {
          digitalWrite(slpPin, LOW); // turn off the motor driver
          Serial.println("Error: motor mode not assigned correctly. End buoyancy compensator...");
          endBuoyancy = HIGH;
      }
    }

    else {
      digitalWrite(slpPin, LOW); // turn off the motor driver
      Serial.println("Error: motor driver not enabled correctly. End buoyancy compensator...");
      endBuoyancy = HIGH;
    }
  }

End No motor driver because no buoyancy compensator*******/


/**********READING SENSOR INPUTS**************/

/***No testing occurring  
 *  void readSensors() {
  // Read the input values (use for troubleshooting input sensors)
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
Testing not currently operational***/
