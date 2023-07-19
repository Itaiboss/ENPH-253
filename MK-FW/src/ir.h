/**
 * @file logs.h
 * @brief contains methods for IR sensing and convolution
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */

#include <stdint.h>
#include <Wire.h>
#pragma once
#define IR_READ PA_0    //anlog read pin
#define IR_RESET PA12    //digital read pin
#define NUM_SAMPLES 128 // number of samples which are being collected.
#define PERIODS 2

void ir_init();

uint32_t ir_sample();
