// Test code for IMU(MPU6050) and bar02 pressure sensor (MS5837)
// SUBC: the UBC Submarine Design Team

// The user must add the libraries "Sketch > Include Libaray > Add .ZIP Library..."
// and select I2Cdev.zip, MPU6050.zip and BlueRobotics_MS5837_Library-master.zip
// These zip files are located under "SUBC_ELEC_2018_19\custom_libraries_4_arduino" of the repo.
 
// Libraries for communicating with IMU
#include <Wire.h>
#include "I2Cdev.h"
#include <MPU6050.h>
// Buoyancy libraries
#include <MS5837.h>

// IMU variables and objects
MPU6050 sensor;
bool sensorWorking = false;
int16_t ax, ay, az, oldAX=90;
int16_t gx, gy, gz;

// Buoyancy sensor
MS5837 depthSensor;


void setup(){
  Serial.begin(9600);
  Serial.println("Starting");

  // Initialize serial monitoring
  
 while (!Serial) // Waiting for serial connection
    {
    }

  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  
  Wire.begin();
  
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s)."); 

  // Initialize IMU
 
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
  
  // Initialize buoyancy sensor
  depthSensor.setModel(MS5837::MS5837_02BA);
  depthSensor.init();
  //if (!depthSensor.init()){
  //  Serial.println("Buoyancy sensor initialization failed!");
  //  delay(1000);
  //}
  depthSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}


void loop(){
  // IMU printout
    sensor.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);

    ax = map (ax, -17000, 21000, 0, 180) ;
    Serial.print(" AutomatedY:");
    Serial.print(ax);

    Serial.println("");

  // Pressure sensor printout
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
  delay(1000);

}
