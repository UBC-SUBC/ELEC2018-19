#include <Wire.h>

//const int MPU_addr=0x69;
const int MPU_addr=0x68; //This is the address somebody suggested changing if we want to have more than one thing connected to the same port
                         //I think they suggested changing it to 0x69
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265;
int maxVal=402;

double x;
double y;
double z;
 
double xave;
double yave;
double zave;

String ledx;
String ledy;
String ledz;

void setup(){
  init();
  //Setup for the OLED
  //SeeedGrayOled.init(SH1107G);           //initialze SEEED OLED display
  //SeeedGrayOled.clearDisplay();          //clear the screen and set start position to top left corner
  //SeeedGrayOled.setNormalDisplay();      //Set display to normal mode (i.e non-inverse mode) Uncomment this and comment the line below if you want this mode
  
  //This stuff is to print the XYZ tilts on the OLED
  //SeeedGrayOled.setTextXY(1, 3);
  //SeeedGrayOled.putString("X");
  //SeeedGrayOled.setTextXY(3, 3);
  //SeeedGrayOled.putString("Y");
  //SeeedGrayOled.setTextXY(5, 3);
  //SeeedGrayOled.putString("Z");

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
}
  
void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);

       x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
       y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
       z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

     int i;
     for (i=0; i<9; i++){
      xave += x;
      yave += y;
      zave += z;
      delay(50);
     }
     xave = xave/10;
     yave = yave/10;
     zave = zave/10;

     Serial.print("AngleX= ");
     Serial.println(x);
     //ledx = String(xave, DEC);
    // SeeedGrayOled.setTextXY(1, 1);
     //SeeedGrayOled.putString(&ledx);

     Serial.print("AngleY= ");
     Serial.println(y);
     ledy = String(yave, 4);
     //SeeedGrayOled.setTextXY(3, 1);
     //SeeedGrayOled.putString(&ledy);

     Serial.print("AngleZ= ");
     Serial.println(z);
     Serial.println("-----------------------------------------");
     ledz = String(zave, 4);
     //SeeedGrayOled.setTextXY(5, 1);
     //SeeedGrayOled.putString(&ledz);
}
