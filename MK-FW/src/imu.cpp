#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
const int MPU = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
const int thresh = 7000;
int16_t lastAcX, lastAcY, lastAcZ;
int16_t lastGyX, lastGyY, lastGyZ;
int16_t diffAcX, diffAcY, diffAcZ = -1;
int16_t offsetAcX, offsetAcY, offsetAcZ = 0;
int16_t offsetGyX, offsetGyY, offsetGyZ = 0;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;
 
void imuInit(){

Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(9600);

}
bool isOnRocks(){

bool returnVal = false;

Wire.beginTransmission(MPU);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU,14,true);
AcX=(Wire.read()<<8|Wire.read());
AcY=(Wire.read()<<8|Wire.read());
AcZ=(Wire.read()<<8|Wire.read());

if(diffAcX == -1){
    diffAcX = 0;
    diffAcY = 0;
    diffAcZ = 0;
}
else{
diffAcX = abs(AcX - lastAcX);
diffAcY = abs(AcY - lastAcY);
diffAcZ = abs(AcZ - lastAcZ);
}

if(diffAcX > thresh || diffAcY > thresh || diffAcZ > thresh){
    returnVal = true;
}

Wire.beginTransmission(MPU);
Wire.write(0x43);
Wire.endTransmission(false);
Wire.requestFrom(MPU,6,true);
GyX=(Wire.read()<<8|Wire.read());
GyY=(Wire.read()<<8|Wire.read());
GyZ=(Wire.read()<<8|Wire.read());

lastAcX = AcX;
lastAcY = AcY;
lastAcZ = AcZ;
lastGyX = GyX;
lastGyY = GyY;
lastGyZ = GyZ;
 
delay(1000);

return returnVal;
}

void imuZero(){

    offsetAcX=lastAcX;
    offsetAcY=lastAcY;
    offsetAcZ=lastAcZ;
    offsetGyX=lastGyX;
    offsetGyY=lastGyY;
    offsetGyZ=lastGyZ;

}

void imuGetPosition(){

    int xAng = map(lastAcX,minVal,maxVal,-90,90);
    int yAng = map(lastAcY,minVal,maxVal,-90,90);
    int zAng = map(lastAcZ,minVal,maxVal,-90,90);
    
    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

}