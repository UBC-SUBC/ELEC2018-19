/*
 Example using the SparkFun HX711 breakout board with a scale
 By: Nathan Seidle
 SparkFun Electronics
 Date: November 19th, 2014
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 This example demonstrates basic scale output. See the calibration sketch to get the calibration_factor for your
 specific load cell setup.
 
 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE
 
 The HX711 does one thing well: read load cells. The breakout board is compatible with any wheat-stone bridge
 based load cell which should allow a user to measure everything from a few grams to tens of tons.
 Arduino pin 2 -> HX711 CLK
 3 -> DAT
 5V -> VCC
 GND -> GND 
 
 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.
 
*/

#include "HX711.h"

#define calibration_factor -10860.0 //calibrated with 750g test load

#define DOUT  3
#define CLK  2

HX711 scale;

void setup() {
  Serial.begin(9600);
  
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor); 
  scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0
}

void loop() {
  Serial.print(scale.get_units(), 2); //"scale.get_units(), n" returns a float with n sigfigs
  Serial.println(" N"); //Units can change but you'll need to refactor the calibration_factor
}
