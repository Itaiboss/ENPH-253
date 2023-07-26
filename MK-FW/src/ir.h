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
#define MOTOR_SLOW_SPEED 0.5
#define MOTOR_MAX_SPEED 0.85

void ir_init();

bool IR_present();

void ir_PID();

int32_t get_error(uint32_t right, uint32_t left, uint32_t right_extreme, uint32_t left_extreme);
double normalize_magnitude(double total, uint32_t ampltitude);

void resetMaximums();

void resetErrors();

enum sensors { left, right, extreme_left, extreme_right };
