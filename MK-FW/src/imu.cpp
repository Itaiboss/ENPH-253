#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MPU6050_light.h>
 
MPU6050 mpu(Wire);
int16_t roll, pitch, yaw;
int16_t lastRoll, lastPitch, lastYaw;
int16_t offsetRoll, offsetPitch, offsetYaw = 0;
int16_t threshold = 5; //degrees, change of angle that happens when car is on rocks

void imuInit(){
    byte status = mpu.begin();
    mpu.calcOffsets();
}

void getPosition(){
    lastRoll = roll;
    lastPitch = pitch;
    lastYaw = yaw;

    mpu.update();

    roll = mpu.getAngleX();
    pitch = mpu.getAngleY();
    yaw = mpu.getAngleZ();

}

bool isOnRocks(){
    int16_t diffRoll, diffPitch, diffYaw;
    diffRoll = abs(roll - lastRoll);
    diffPitch = abs(pitch - lastPitch);
    diffYaw = abs(yaw - lastYaw);
    if(diffRoll >= threshold || diffPitch >= threshold){// || diffYaw >= threshold){
        return true;
    }
    return false;
}

void storePosition(){
    offsetRoll = roll;
    offsetPitch = pitch;
    offsetYaw = yaw;
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
