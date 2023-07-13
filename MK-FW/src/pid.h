/**
 * @file      pid.cpp
 * @brief     contains algorithm for PID control
 *
 */
#include <stdint.h>
#define TAPE_R PA12
#define TAPE_L PA11

void pidInit();
uint32_t PID();
