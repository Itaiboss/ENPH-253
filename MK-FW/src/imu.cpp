#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
const int MPU = 0x68;
int16_t AcX, AcY, AcZ;
const int thresh = 7000;
int16_t lastX, lastY, lastZ;
int16_t diffX, diffY, diffZ = -1;
int16_t offsetX, offsetY, offsetZ = 0;
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

if(diffX == -1){
    diffX = 0;
    diffY = 0;
    diffZ = 0;
}
else{
diffX = abs(AcX - lastX);
diffY = abs(AcY - lastY);
diffZ = abs(AcZ - lastZ);
}

if(diffX > thresh || diffY > thresh || diffZ > thresh){
    returnVal = true;
}

lastX = AcX;
lastY = AcY;
lastZ = AcZ;

Serial.println(AcX);
Serial.println(AcY);
Serial.println(AcZ);
 
delay(1000);

return returnVal;
}

void imuZero(){

    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,6,true);
    offsetX=(Wire.read()<<8|Wire.read());
    offsetY=(Wire.read()<<8|Wire.read());
    offsetZ=(Wire.read()<<8|Wire.read());

}

void imuGetPosition(){
    
    int xAng = map(lastX,minVal,maxVal,-90,90);
    int yAng = map(lastY,minVal,maxVal,-90,90);
    int zAng = map(lastZ,minVal,maxVal,-90,90);
    
    x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

}