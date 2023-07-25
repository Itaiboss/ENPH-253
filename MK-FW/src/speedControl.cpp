#include <pins.h>
#include <speedControl.h>
#include <wire.h>


volatile double motorSpeed = 0;


/**
 * @brief sets the motor speed to the duty cycle of the percentage of the double.
 * *  Negative values correspond the backwards direction and positve the the forward direction. 
 * @note   
 * @param  speed: the duty cycle percentage you would like to power your motor at
 * @retval None
 */
void set_motor_speed(double speed) {
    if (speed > 0) {
        pwm_start(MOTOR_1A, MOTOR_CONTROL_FREQUENCY, speed / 4098, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(MOTOR_2A, MOTOR_CONTROL_FREQUENCY, speed / 4098, RESOLUTION_12B_COMPARE_FORMAT);
    } else if (speed < 0) {
        pwm_start(MOTOR_1B, MOTOR_CONTROL_FREQUENCY, -speed / 4098, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(MOTOR_2B, MOTOR_CONTROL_FREQUENCY, -speed / 4098, RESOLUTION_12B_COMPARE_FORMAT);
    } else {
        pwm_start(MOTOR_1B, MOTOR_CONTROL_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(MOTOR_2B, MOTOR_CONTROL_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(MOTOR_1A, MOTOR_CONTROL_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(MOTOR_2A, MOTOR_CONTROL_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    }

    speed = motorSpeed;
}

/**
 * @brief The difference where positive refers to increasing motor 1 and negative refers to decreasing motor 2.
 * @note   Assumes the device is currently moving forward. 
 * @param  difference: 
 * @retval None
 */
void set_differential_steering(double difference) {
    double top_motor_1_speed = motorSpeed + difference / 2;
    double top_motor_2_speed = motorSpeed - difference / 2;

    pwm_start(MOTOR_1A, MOTOR_CONTROL_FREQUENCY, top_motor_1_speed / 4098, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(MOTOR_2A, MOTOR_CONTROL_FREQUENCY, top_motor_2_speed / 4098, RESOLUTION_12B_COMPARE_FORMAT);

}



