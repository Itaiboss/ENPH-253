#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MPU6050_light.h>
#include <pins.h>
 
MPU6050 mpu(Wire);
int16_t roll, pitch, yaw;
int16_t lastRoll, lastPitch, lastYaw;
int16_t offsetRoll, offsetPitch, offsetYaw = 0;
int16_t threshold = 3; //degrees, change of angle that happens when car is on rocks

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

/**
 * @brief  Determines if the object is on the rocks. It is important to
 * @attention This function uses the last values saved when calling `getPosition()`. 
 * @note   Assumes that roll and pitch are not the rotation about the vertical axis. 
 * @retval True if the robot is on the rocks, false otherwise. 
 */
bool isOnRocks(){
    int16_t diffRoll, diffPitch, diffYaw;
    diffRoll = abs(roll - lastRoll);
    diffPitch = abs(pitch - lastPitch);
    diffYaw = abs(yaw - lastYaw);
    if(diffPitch >= threshold){// || diffYaw >= threshold){
        return true;
    }
    return false;
}

/**
 * @brief  stores the current position to be the zero value. 
 * @attention   You need not call getPosition() before zeroing the values. 
 * @retval None
 */
void storePosition(){
    int32_t roll_total;
    int32_t yaw_total;
    int32_t pitch_total;

    for (int i = 0; i < 6; i++) {
        getPosition();
        // roll_total = getRoll();
        // yaw_total = getYaw();
        // pitch_total = getPitch();
    }

    offsetRoll = roll_total / 6;
    offsetPitch = pitch_total / 6;
    offsetYaw = yaw_total / 6;
}

/**
 * @brief  Converts the given angle between -360 and 360 to the domain of -180 and 180. 
 * @note   
 * @param  angle: The angle you would like to convert. 
 * @retval A integer value within -180 and 180 represetning the angle in that domain. 
 */
int16_t convertToDomain(int16_t angle) {
    if (angle > 180) {
        return angle - 360;
    }

    if (angle < -180) {
        return 360 + angle;
    }

    return angle;
}

int16_t getRoll(){
    return convertToDomain(roll - offsetRoll);
}

int16_t getPitch(){
    return convertToDomain(pitch - offsetPitch);
}

int16_t getYaw(){
    return convertToDomain(yaw - offsetYaw);
}


