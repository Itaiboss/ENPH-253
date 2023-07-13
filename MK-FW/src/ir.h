/**
 * @file logs.h
 * @brief contains methods for IR sensing and convolution
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */

#include <stdint.h>
#include <Wire.h>
#pragma once
#define IR_READ PA0    //anlog read pin
#define IR_RESET PA12    //digital read pin

#define ADC_READ_PERIOD 50 // length of time between reads in us.
#define F_READ_PERIOD 1000 // length of time between convultion calculations in us.
#define NUM_SAMPLES 100 // number of samples which are being collected.

void ir_init();

void ir_sample();

int getCorrelationFactor(double waveOne[], uint32_t waveTwo[],int waveOneSize ,int waveTwoSize);