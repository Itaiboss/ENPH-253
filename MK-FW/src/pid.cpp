/**
 * @file      pid.cpp
 * @brief     ENPH 253 competition main 
 *
 * @copyright Copyright ENPH253 (c) 2023
 *
 * @author Itai Boss
 * @author Logan Underwoof
 * @author Imogen neil
 * @author Brooklynn Erikson
 */

#include <pid.h>
#include <stdint.h>
#include <arduino.h>
#pragma once

double kp = 2;
double ki = 0;
double kd = 0;
#define TAPE_R PA_12
#define TAPE_L PA_11
uint32_t target;
int32_t control;
bool sense_r;
bool sense_l;
uint32_t last_r;
uint32_t last_l;
uint32_t time_ms;
uint32_t last_time;
int32_t error;
int32_t total_error;
int32_t last_error;
int32_t d_error;
int32_t max_control=492;
int32_t min_control=82;
const int32_t lookup[2][2]={{2,-1}, {1,0}}; // l,r

uint32_t PID() {
if (time_ms > millis()-last_time) {
    sense_r = digitalRead(TAPE_R);
    sense_l = digitalRead(TAPE_L);
    int32_t error = ((sense_r==sense_l) && (last_l==0))? lookup[sense_l][sense_r]*-1:lookup[sense_l][sense_r];
    total_error += error;
    if (total_error >= max_control) {
        total_error = max_control;
    } else if (total_error <= min_control) {
        total_error = min_control;
    }
    d_error = error-last_error;
    control = kp*error + (ki*time_ms)*total_error + (kd/time_ms)*d_error+246;
    if (control >= max_control) {
        control = max_control;
    } else if (control <= min_control) {
        control = min_control;
    }
    last_error = error;
    last_r = sense_r;
    last_l = sense_l;
    last_time = millis();
    return control;
    }
}

//servo center = 6% duty cycle
//servo max right = 10% duty cycle
//servo max left = 2% duty cycle









