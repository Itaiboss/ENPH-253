#include <pins.h>
#include <stdint.h>

#define MOTOR_CONTROL_FREQUENCY 1000

void set_motor_speed(double speed);

void set_differential_steering(double difference);

void set_steering(double turning_value);