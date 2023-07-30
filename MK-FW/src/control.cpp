#include <control.h>
#include <Wire.h>

/**
 * @brief  Sets the steering for the front wheels. 
 * @note   
 * @param  turning_value: A value between 1000 and -1000 where 1000 refers to maximum left turn and -100 to maximum right turn. 
 * @retval None
 */
void set_steering(int32_t turning_value) {
    if (turning_value > 1000) {
        turning_value = 1000;
    }

    if (turning_value < 1000) {
        turning_value == 1000;
    }

    uint32_t valueToMotor;
    if (turning_value > 0) {
        valueToMotor = (LEFT_MAX - MID_POINT) * (double) turning_value / 1000 + MID_POINT;
    } else if (turning_value < 0) {
        valueToMotor = (MID_POINT - RIGHT_MAX) * (double) turning_value / 1000 + MID_POINT;
    } 

    pwm_start(SERVO, 50, valueToMotor, RESOLUTION_12B_COMPARE_FORMAT);
}


void set_raw_steering(uint32_t pwm_value) {
    pwm_start(SERVO, 50, pwm_value, RESOLUTION_12B_COMPARE_FORMAT);
}

volatile double motorSpeed = 0;
/**
 * @brief sets the motor speed to the duty cycle of the percentage of the double.
 * *  Negative values correspond the backwards direction and positve the the forward direction. 
 * @note   
 * @param  speed: the duty cycle percentage you would like to power your motor at
 * @retval None
 */
void set_motor_speed(int32_t speed) {
    if (speed > 0) {
        pwm_start(LEFT_MOTOR_FORWARD, MOTOR_CONTROL_FREQUENCY, (double) speed / 100 * 4098, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR_FORWARD, MOTOR_CONTROL_FREQUENCY, (double) speed / 100 * 4098, RESOLUTION_12B_COMPARE_FORMAT);
    } else if (speed < 0) {
        pwm_start(LEFT_MOTOR_BACKWARD, MOTOR_CONTROL_FREQUENCY, - (double) speed / 100 * 4098, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR_BACKWARD, MOTOR_CONTROL_FREQUENCY, - (double) speed / 100 * 4098, RESOLUTION_12B_COMPARE_FORMAT);
    } else {
        cut_motors();
    }
    motorSpeed = speed;
}

/**
 * @brief  Turns off the motors. 
 * @note   
 * @retval None
 */
void cut_motors() {
    pwm_start(LEFT_MOTOR_FORWARD, MOTOR_CONTROL_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_MOTOR_FORWARD, MOTOR_CONTROL_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    
    pwm_start(LEFT_MOTOR_BACKWARD, MOTOR_CONTROL_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_MOTOR_BACKWARD, MOTOR_CONTROL_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);

    motorSpeed = 0;
}

/**
 * @brief Sets the differential steering to the provided value.
 * @note   Assumes the device is currently moving forward. It will also never cause the motor to spin backwards. 
 * @param  difference: A percentage difference you would like to see in the motors. A > 0 percentage will refer to the left wheel decreased and < 0 percnetage will refer to right wheel decreasing in speed. 
 * @retval None
 */
void set_differential_steering(int32_t difference) {
    // first double checks to make sure we are moving forward at this moment. 
    if (motorSpeed > 0) {
        if (difference > 0) {
            pwm_start(LEFT_MOTOR_FORWARD, 1000, motorSpeed - (double) difference / 100 * 4098, RESOLUTION_12B_COMPARE_FORMAT);
        } else {
            pwm_start(RIGHT_MOTOR_FORWARD, 1000, motorSpeed + (double) difference / 100 * 4098, RESOLUTION_12B_COMPARE_FORMAT);
        }
        
    }

}

void centre_steering() {
    set_steering(0);
}
