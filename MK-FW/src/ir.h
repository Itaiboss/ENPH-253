/**
 * @file logs.h
 * @brief contains methods for IR sensing and convolution
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */

#include <stdint.h>
#include <Wire.h>
#pragma once

#define NUM_SAMPLES 32 // number of samples which are being collected.
#define BOTTOM_FREQUENCY_CUTOFF 900
#define TOP_FREQUENCY_CUTOFF 1150
#define MIN_DETECTION_AMPLITUDE 100
#define EXPECTED_FREQUENCY
#define IS_CLOSE_VALUE 740

void ir_init();

bool noIRFound();

bool getIsClose();

bool IR_present();

void resetCounter();

void ir_PID();

int32_t get_error(uint32_t right, uint32_t left, uint32_t right_extreme, uint32_t left_extreme);

double normalize_magnitude(double total, uint32_t ampltitude);

void resetMaximums();

void resetErrors();

enum sensors { left, right, extreme_left, extreme_right };
