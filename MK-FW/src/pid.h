/**
 * @file      pid.cpp
 * @brief     contains algorithm for PID control
 *
 */
#include <stdint.h>

#define KP 25
#define KI 0
#define KD 0

#define BLACK_LEFT_CUTOFF 375
#define BLACK_RIGHT_CUTOFF 375
#define ON_ROCKS_LEFT_READ 375
#define ON_ROCKS_RIGHT_READ 375


void resetTotal();
void digitalPID(int32_t kp, int32_t ki, int32_t kd);
void analogPID(double kp, double kd, double ki, uint32_t left_threshold, uint32_t right_threshold);
void pidInit();
void PID(int32_t kp, int32_t ki, int32_t kd);
bool tapeIsPresent();

