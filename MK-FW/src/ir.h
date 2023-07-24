/**
 * @file logs.h
 * @brief contains methods for IR sensing and convolution
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */

#include <stdint.h>
#include <Wire.h>
#pragma once
#define IR_READ_RIGHT PA_0   //anlog read pin
#define IR_READ_LEFT PA_1
#define IR_READ_EXTREME_LEFT PA_4   //anlog read pin
#define IR_READ_EXTREME_RIGHT PA_5
#define IR_RESET PA12    //digital read pin
#define NUM_SAMPLES 128 // number of samples which are being collected.
#define ROLLING_AVERAGE_NUMBER 5

void ir_init();

uint32_t ir_PID();

uint32_t ir_follow_steering_value();

double get_error(uint32_t right, uint32_t left);
double normalize_magnitude(double total, uint32_t ampltitude);

void resetMaximums();

enum sensors { left, right, extreme_left, extreme_right };
