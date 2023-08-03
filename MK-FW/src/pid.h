/**
 * @file      pid.cpp
 * @brief     contains algorithm for PID control
 *
 */
#include <stdint.h>

#define KP 30
#define KI 0
#define KD -5


void pidInit();
void PID(int32_t kp, int32_t ki, int32_t kd);
bool tapeIsPresent();

