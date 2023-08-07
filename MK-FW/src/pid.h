/**
 * @file      pid.cpp
 * @brief     contains algorithm for PID control
 *
 */
#include <stdint.h>

#define KP 25
#define KI 0
#define KD 0

#define BLACK_LEFT_CUTOFF 340
#define BLACK_RIGHT_CUTOFF 340
#define ON_ROCKS_LEFT_READ 300
#define ON_ROCKS_RIGHT_READ 300


void resetTotal();
void digitalPID(int32_t kp, int32_t ki, int32_t kd);
void analogPID(int32_t kp, int32_t ki, int32_t kd);
void pidInit();
void PID(int32_t kp, int32_t ki, int32_t kd);
bool tapeIsPresent();

