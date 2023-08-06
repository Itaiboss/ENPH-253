/**
 * @file      pid.cpp
 * @brief     contains algorithm for PID control
 *
 */
#include <stdint.h>

#define KP 25
#define KI 0
#define KD 0


void pidInit();
void PID(int32_t kp, int32_t ki, int32_t kd);
bool tapeIsPresent();

