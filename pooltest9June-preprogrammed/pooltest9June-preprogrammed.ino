#include <Servo.h>

#define SERVOPIN 9

Servo theServo;
int i;

void slowMove(int, int);

void setup(){
  theServo.attach(SERVOPIN);
  theServo.write(90);
  Serial.begin(9600);
}

void loop(){
  slowMove(90, 120);
  delay(1000);
  slowMove(120, 90);
  delay(2000);
  slowMove(90, 60);
  delay(1000);
  slowMove(60, 90);
  delay(2000);
}

void slowMove(int oldPos, int newPos){
  int difference = newPos-oldPos;
  /*for (i=0;i<75;i++){
    theServo.write(oldPos+(i*difference/75));
    Serial.println(oldPos+(i*difference/75));
    delay(10);
  }*/
  theServo.write(newPos);
}
