#include <pins.h>
#include <stdint.h>

#define MOTOR_CONTROL_FREQUENCY 1000

void set_motor_speed(int32_t speed);
void cut_motors();
void set_differential_steering(int32_t difference);

void set_steering(int32_t turning_value);
void set_raw_steering(uint32_t pwm_value);
void spin_in_circle(bool right);
void centre_steering();
void cut_servo();
