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
uint32_t target;
int32_t control;
uint32_t sense_r;
uint32_t sense_l;
uint32_t sense_rr;
uint32_t sense_ll;
uint32_t last_r;
uint32_t last_l;
uint32_t time_ms = 8;
uint32_t last_time = 0;
int32_t error = 0;
int32_t total_error;
int32_t last_error;
int32_t d_error;
int32_t max_control = LEFT_MAX;
int32_t min_control = RIGHT_MAX;
bool diff = 0;
int32_t integral_max = 250;
int32_t integral_min = 250;
const int32_t error_table[9]={9,4,2,1,0,1,-2,-4,-9,}; 

void pidInit() {
    pinMode(TAPE_L, INPUT_PULLUP);
    pinMode(TAPE_R, INPUT_PULLUP);
    pinMode(TAPE_E_L, INPUT_PULLUP);
    pinMode(TAPE_E_R, INPUT_PULLUP);
    pinMode(TAPE_RR, INPUT_PULLUP);
    pinMode(TAPE_LL, INPUT_PULLUP);
}

void PID( int32_t kp, int32_t ki, int32_t kd) {
    error = 0;
    diff = false; 
    //if (millis()- last_time > time_ms) {
        diff = 0;
        sense_r = digitalRead(TAPE_R);
        sense_l = digitalRead(TAPE_L);
        sense_rr = digitalRead(TAPE_RR);
        sense_ll = digitalRead(TAPE_LL);
        
        //0110
        if( !sense_ll && sense_l && sense_r && !sense_rr){
            error = error_table[4];
        //1110
        } else if( sense_ll && sense_l && sense_r && !sense_rr){
            error = error_table[3];
        //0111
        }else if( !sense_ll && sense_l && sense_r && sense_rr){
            error = error_table[5];
        //0011
        } else if( !sense_ll && !sense_l && sense_r && sense_rr){
            error = error_table[6];
        } 
        //1100
         else if( sense_ll && sense_l && !sense_r && !sense_rr){
            error = error_table[2];
        //1000
        } else if( sense_ll && !sense_l && !sense_r && !sense_rr){
            error = error_table[1];
        //0001
        } else if( !sense_ll && !sense_l && !sense_r && sense_rr){
            error = error_table[7];
        //0000
        } else if( !sense_ll && !sense_l && !sense_r && !sense_rr){
            if (last_error < 0) {
                error = error_table[8];
                // set_differential_steering(-20);
                diff = true;
            } else if (last_error > 0){
                error = error_table[0];
                // set_differential_steering(20);
                diff =true; 
            }
        // } else if (sense_ll && sense_l && sense_r && sense_rr) {
        //     error = 0;
        } else {
            error = last_error;
        }
        if (!diff) {
            // set_differential_steering(0);
        }
        //CONSOLE_LOG(LOG_TAG,"%d,%d,%d,%d", sense_ll,sense_l, sense_r, sense_rr);
        // CONSOLE_LOG(LOG_TAG,"%d",error);
        total_error += error;
        d_error = error-last_error;
        control = kp*error + (ki*time_ms)*total_error + (kd/time_ms)*d_error + MID_POINT;
        // CONSOLE_LOG(LOG_TAG,"%i",control);
        if (control >= max_control) {
            control = max_control;
        } else if (control <= min_control) {
            control = min_control;
        }

        if (abs(error) > 2) {
            digitalWrite(PC13, HIGH);
        } else {
            digitalWrite(PC13, LOW);
        }
        // CONSOLE_LOG(LOG_TAG,"%i",control);
        last_error = error;
        last_r = sense_r;
        last_l = sense_l;
        // CONSOLE_LOG(LOG_TAG, "Time: %d",millis()-last_time);
        last_time = millis();
        // set_raw_steering(control);
    //}
}
bool tapeIsPresent() {
    return digitalRead(TAPE_L) || digitalRead(TAPE_R);
}

//servo center = 6% duty cycle 246
//servo max right = 10% duty cycle 490
//servo max left = 2% duty cycle 82

//steering bound 100-400








