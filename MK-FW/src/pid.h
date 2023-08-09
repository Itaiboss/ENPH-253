/**
 * @file      pid.cpp
 * @brief     contains algorithm for PID control
 *
 */
#include <stdint.h>

#define KP 25
#define KI 0
#define KD 0

#define BLACK_LEFT_CUTOFF 250
#define BLACK_RIGHT_CUTOFF 250
#define ON_ROCKS_LEFT_READ 400
#define ON_ROCKS_RIGHT_READ 400


void resetTotal();
void digitalPID(int32_t kp, int32_t ki, int32_t kd);
void analogPID(double kp, double ki, double kd, uint32_t white_threshold);
void pidInit();
void PID(int32_t kp, int32_t ki, int32_t kd);
bool tapeIsPresent();

