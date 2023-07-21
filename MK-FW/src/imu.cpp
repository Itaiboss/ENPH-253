#include <Wire.h>
#include <math.h>
 
#define SIZE 50
const int MPU = 0x68;
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
const int thresh = 7000;
int16_t lastAcX, lastAcY, lastAcZ;
int16_t diffAcX, diffAcY, diffAcZ = -1;
int16_t offsetRollVelocity, offsetPitchVelocity, offsetYawVelocity;
int16_t offsetRoll, offsetPitch, offsetYaw;
int16_t roll, pitch, yaw;
int16_t rollVelocity, pitchVelocity, yawVelocity;
int minVal=265;
int maxVal=402;
 
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
AcX=(Wire.read()<<8|Wire.read()) ;
AcY=(Wire.read()<<8|Wire.read()) ;
AcZ=(Wire.read()<<8|Wire.read()) ;

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

return returnVal;
}

void imuZero(){
    offsetRollVelocity=GyX;
    offsetPitchVelocity=GyY;
    offsetYawVelocity=GyZ;
    offsetRoll = roll;
    offsetPitch = pitch;
    offsetYaw = yaw;
}

void getPosition(){
    int16_t r, p, y, rV, pV, yV = 0;

    for(int i = 0; i < SIZE; i++){
        isOnRocks();
        int xAng = map(lastAcX,minVal,maxVal,-90,90);
        int yAng = map(lastAcY,minVal,maxVal,-90,90);
        int zAng = map(lastAcZ,minVal,maxVal,-90,90);
        
        r += (RAD_TO_DEG * (atan2(-yAng, -zAng)+PI));
        p += (RAD_TO_DEG * (atan2(-xAng, -zAng)+PI));
        y += (RAD_TO_DEG * (atan2(-yAng, -xAng)+PI));

        rV += GyX;
        pV += GyY;
        yV += GyZ;
    }

    roll = r / SIZE;
    pitch = p / SIZE;
    yaw = y / SIZE;

    rollVelocity = rV / SIZE;
    pitchVelocity = pV / SIZE;
    yawVelocity = yV / SIZE;
}

int16_t getRoll(){
    return roll - offsetRoll;
}

int16_t getPitch(){
    return pitch - offsetPitch;
}

int16_t getYaw(){
    return yaw - offsetYaw;
}

int16_t getRollVelocity(){
    return rollVelocity - offsetRollVelocity;
}

int16_t getPitchVelocity(){
    return pitchVelocity - offsetPitchVelocity;
}

int16_t getYawVelocity(){
    return yawVelocity - offsetYawVelocity;
}
