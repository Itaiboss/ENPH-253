/**
 * @file      pid.cpp
 * @brief     contains algorithm for PID control
 *
 */

#include <pid.h>
#include <stdint.h>
#include <arduino.h>
#include <logs.h>
#include <pins.h>
#include <control.h>
#pragma once

static const char* LOG_TAG = "PID";

double kp = 60;
double ki = 0;//0.005;
double kd = 0;//2;
uint32_t target;
int32_t control;
uint32_t sense_r;
uint32_t sense_l;
uint32_t sense_rr;
uint32_t sense_ll;
uint32_t last_r;
uint32_t last_l;
uint32_t time_ms = 10;
uint32_t last_time = 0;
int32_t error;
int32_t total_error;
int32_t last_error;
int32_t d_error;
int32_t max_control = LEFT_MAX;
int32_t min_control = RIGHT_MAX;
int32_t diff = 0;
int32_t integral_max = 250;
int32_t integral_min = 250;
const double lookup[2][2] = {{5,1}, {-1,0}}; // l,r

void pidInit() {
    pinMode(TAPE_L, INPUT_PULLUP);
    pinMode(TAPE_R, INPUT_PULLUP);
    pinMode(TAPE_E_L, INPUT_PULLUP);
    pinMode(TAPE_E_R, INPUT_PULLUP);
    pinMode(TAPE_RR, INPUT_PULLUP);
    pinMode(TAPE_LL, INPUT_PULLUP);
}

void PID() {
    if (millis()> last_time + time_ms) {
        diff=0;
        sense_r = digitalRead(TAPE_R);
        sense_l = digitalRead(TAPE_L);
        sense_rr = digitalRead(TAPE_RR);
        sense_ll = digitalRead(TAPE_LL);
        CONSOLE_LOG(LOG_TAG,"TAPE LL:%d TAPE L:%d TAPE R:%d TAPE RR:%d", sense_ll,sense_l, sense_r, sense_rr);
        int32_t error = 0;
        if (sense_r == 0 && sense_l == 0) {
            if (last_l == 1) {
                error = lookup[sense_r][sense_l];
            } else if (last_r == 1) {
                error = lookup[sense_r][sense_l]*-1;
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
        control = kp*error + (ki*time_ms)*total_error + (kd/time_ms)*d_error + MID_POINT;
        if (control >= max_control) {
            diff = (max_control-control)*-1;
            CONSOLE_LOG(LOG_TAG,"%d",diff);
            control = max_control;
        } else if (control <= min_control) {
            diff = (control-min_control);
            CONSOLE_LOG(LOG_TAG,"%d",diff);
            control = min_control;
        }
        //CONSOLE_LOG(LOG_TAG,"%d",control);
        last_error = error;
        last_r = sense_r;
        last_l = sense_l;
        last_time = millis();
        }
    pwm_start(SERVO, 50, control, RESOLUTION_12B_COMPARE_FORMAT);
    if (diff != 0) {
        if (diff > 0) {
            //set_differential_steering(.5, true);
        }
        if (diff < 0) {
            //set_differential_steering(.5, false);
        }
    } else {
        set_motor_speed(65);
    }
}

bool tapeIsPresent() {
    return digitalRead(TAPE_L) || digitalRead(TAPE_R);
}

//servo center = 6% duty cycle 246
//servo max right = 10% duty cycle 490
//servo max left = 2% duty cycle 82

//steering bound 100-400








