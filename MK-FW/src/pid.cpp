/**
 * @file      pid.cpp
 * @brief     contains algorithm for PID control
 *
 */

#include <pid.h>
#include <stdint.h>
#include <arduino.h>
#include <logs.h>
#pragma once

static const char* LOG_TAG = "PID";

double kp = 70;
double ki = 0;
double kd = 200;
uint32_t target;
int32_t control;
uint32_t sense_r;
uint32_t sense_l;
uint32_t last_r;
uint32_t last_l;
uint32_t time_ms = 17;
uint32_t last_time = 0;
int32_t error;
int32_t total_error;
int32_t last_error;
int32_t d_error;
int32_t max_control=492;
int32_t min_control=82;
const int32_t lookup[2][2]={{2,-1}, {1,0}}; // l,r

void pidInit() {
    pinMode(TAPE_L, INPUT_PULLUP);
    pinMode(TAPE_R, INPUT_PULLUP);
}

uint32_t PID() {
    if (millis()> last_time + time_ms) {
        sense_r = digitalRead(TAPE_R);
        sense_l = digitalRead(TAPE_L);
        CONSOLE_LOG(LOG_TAG,"TAPE L:%d TAPE R:%d", sense_l, sense_r);
        int32_t error =0;
        if (sense_r == 0 && sense_l == 0) {
            if (last_l == 1) {
                error = lookup[sense_r][sense_l] *-1;
            } else if (last_r == 1) {
                    error = lookup[sense_r][sense_l];
            } else {
                error = last_error;
            }
        }
        else {
            error = lookup[sense_r][sense_l];
        }
        CONSOLE_LOG(LOG_TAG,"%d",error);
        total_error += error;
        if (total_error >= max_control) {
            total_error = max_control;
        } else if (total_error <= min_control) {
            total_error = min_control;
        }
        d_error = error-last_error;
        control = kp*error + (ki*time_ms)*total_error + (kd/time_ms)*d_error+200;
        CONSOLE_LOG(LOG_TAG,"%d",control);
        if (control >= max_control) {
            control = max_control;
        } else if (control <= min_control) {
            control = min_control;
        }
        last_error = error;
        last_r = sense_r;
        last_l = sense_l;
        last_time = millis();
        }
    return control;
}

//servo center = 6% duty cycle 246
//servo max right = 10% duty cycle 490
//servo max left = 2% duty cycle 82









