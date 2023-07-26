#include <pins.h>
#include <stdint.h>

#define MOTOR_CONTROL_FREQUENCY 1000

void set_motor_speed(double speed, bool forward);

void set_differential_steering(double difference, bool right);

void set_steering(double turning_value, bool right);
void set_steering(uint32_t pwm_value);