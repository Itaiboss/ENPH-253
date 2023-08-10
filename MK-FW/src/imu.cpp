#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MPU6050_light.h>
#include <pins.h>
#include <logs.h>
 
MPU6050 mpu(Wire);
int32_t roll, pitch, yaw;
int32_t lastRoll, lastPitch, lastYaw;
int32_t offsetRoll, offsetPitch, offsetYaw = 0;
int32_t threshold = 2; //degrees, change of angle that happens when car is on rocks

static const char* LOG_TAG = "IMU";

/**
 * @brief  Initilizes the imu by calculating the offsets. 
 * @note   It is essential that the IMU does not move during the operation of this function. 
 * @retval None
 */
void imuInit(){
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
    Wire.begin();
    byte status = mpu.begin();
    mpu.calcOffsets();
}


/**
 * @brief  Collects and saves the current position of the IMU. 
 * @note   This operation is required to be called before many of the functions in this wrapper to get accurate results.
 * @retval None
 */
void getPosition(){


    lastRoll = roll;
    lastPitch = pitch;
    lastYaw = yaw;

    mpu.update();

    roll = mpu.getAngleX();
    pitch = mpu.getAngleY();
    yaw = mpu.getAngleZ();

}

/**
 * @brief  Determines if the current acceleration is upwards. 
 * @note   This function assumes that the x direction is the upwards direction
 * @retval True if the current acceleration is upwards. 
 */
bool isUpwardsAcceleration() {
    if (mpu.getAccZ() > 0) {
        return true;
    }

    return false;
}

double getUpwardsAcc() {
    return mpu.getAccZ();
}

/**
 * @brief  Converts the given angle between -360 and 360 to the domain of -180 and 180. 
 * @note   
 * @param  angle: The angle you would like to convert. 
 * @retval A integer value within -180 and 180 represetning the angle in that domain. 
 */
int32_t convertToDomain(double angle) {
    if (angle > 180) {
        return angle - 360;
    }

    if (angle < -180) {
        return 360 + angle;
    }

    return angle;
}


/**
 * @brief  Determines if the object is on the rocks. It is important to
 * @attention This function uses the last values saved when calling `getPosition()`. 
 * @note   Assumes that roll and pitch are not the rotation about the vertical axis. 
 * @retval True if the robot is on the rocks, false otherwise. 
 */
bool isOnRocks(){
    int32_t diffRoll, diffPitch, diffYaw;
    diffPitch = abs(pitch - lastPitch);
    CONSOLE_LOG(LOG_TAG, "diff_pitch: %i, pitch: %i, last pitch: %i", diffPitch, pitch, lastPitch);
    if(diffPitch >= threshold){// || diffYaw >= threshold){
        return true;
    }
    return false;
}

int32_t getRoll(){
    return convertToDomain(roll - offsetRoll);
}

int32_t getPitch(){
    return convertToDomain(pitch - offsetPitch);
}

int32_t getYaw(){
    return convertToDomain(yaw - offsetYaw);
}

/**
 * @brief  stores the current position to be the zero value. 
 * @attention   You need not call getPosition() before zeroing the values. 
 * @retval None
 */
void storePosition(){

    getPosition();

    offsetRoll = getRoll();
    offsetPitch = getPitch();
    offsetYaw = getYaw();
}





