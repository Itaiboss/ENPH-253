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
double error = 0;
double total_error;
double last_error;
double d_error;
int32_t max_control = LEFT_MAX;
int32_t min_control = RIGHT_MAX;
bool diff = 0;
int32_t integral_max = 250;
int32_t integral_min = 250;
const int32_t error_table[9]={9,4,2,1,0,1,-2,-4,-9,}; 
const double lookup[2][2] = {{5,-1}, {1,0}};

const uint32_t max_error = 900;

// defined constants for the min and max values
const uint32_t to_white_left = 340;
const uint32_t from_white_left = 380;
const uint32_t to_white_right = 220;
const uint32_t from_white_right = 260;
const double right_reading_boost = 1;
const double left_reading_boost = 1;

bool right_white = true;
bool left_white = true;

void pidInit() {
    pinMode(TAPE_L, INPUT);
    pinMode(TAPE_R, INPUT);
    pinMode(TAPE_E_L, INPUT_PULLUP);
    pinMode(TAPE_E_R, INPUT_PULLUP);
    right_white = true;
    left_white = true;
}

void resetTotal() {
    total_error = 0;
}

void analogPID(double kp, double kd, double ki) {
    uint32_t left_reading = analogRead(TAPE_L);
    uint32_t right_reading = analogRead(TAPE_R);

    uint32_t left_min = left_white ? from_white_left : to_white_left; 
    uint32_t right_min = right_white ? from_white_right : to_white_right;

    // if (left_white) {
    //     if (left_reading >= 500) {
    //         left_reading = 0;
    //     }
    // }

    // if (right_white) {
    //     if (right_reading >= 500) {
    //         right_reading = 0;
    //     }
    // }

    

    double total_readings = left_reading + right_reading - left_min - right_min;

    if (left_reading < left_min && right_reading < right_min) {
        error = last_error;
        left_white = true;
        right_white = true;

    }
    else if(left_reading < left_min)  // 00||
    {
        error = -1;
        left_white = true;
        right_white = false;
    } 
    else if (right_reading < right_min)  //||00
    {
        error = 1;
        left_white = false;
        right_white = true;
        // maybe try 1.5 here. 
    } 
    else 
    {
        error = (-right_reading_boost * (right_reading - right_min) + left_reading_boost * (left_reading - left_min)) / total_readings;
        left_white = false;
        right_white = false;
    }


    

   



    total_error += error;
    d_error = error - last_error;
    last_error = error;

    uint32_t steering_value = MID_POINT + kp * error + kd * d_error + ki * total_error;

    if (steering_value >= max_control) {
        steering_value = max_control;
    } else if (steering_value <= min_control) {
        steering_value = min_control;
    } 

    

    if (error == 1) {
        pwm_start(LEFT_MOTOR_FORWARD, 1000, 0, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR_FORWARD, 1000, 3800, RESOLUTION_12B_COMPARE_FORMAT);
    } else if (error == -1) {
        pwm_start(LEFT_MOTOR_FORWARD, 1000, 3800, RESOLUTION_12B_COMPARE_FORMAT);
        pwm_start(RIGHT_MOTOR_FORWARD, 1000, 0, RESOLUTION_12B_COMPARE_FORMAT);
    } else {
        set_motor_speed(getMotorSpeed());
    }

    set_raw_steering(steering_value);
    

    CONSOLE_LOG(LOG_TAG, "%i, [%i], [%i, %i]", steering_value, (int) error * 100, left_reading, right_reading);

    
}

void digitalPID(int32_t kp, int32_t ki, int32_t kd) {

    uint32_t left_reading = analogRead(TAPE_L);
    uint32_t right_reading = analogRead(TAPE_R);

    sense_r = right_reading > BLACK_RIGHT_CUTOFF;
    sense_l = left_reading > BLACK_LEFT_CUTOFF;



    int32_t error = 0;
    if (sense_r == 0 && sense_l == 0) {
        if (last_l == 1) {
            error = lookup[sense_r][sense_l]*-1;
        } else if (last_r == 1) {
            error = lookup[sense_r][sense_l];
        } else {
            error = last_error;
        }
    } else {
        error = lookup[sense_r][sense_l];
    }

    total_error += error;
    uint32_t diff_error = error - last_error;
    uint32_t steering_value = MID_POINT + kp * error + kd * diff_error + ki * total_error;

    if (steering_value >= max_control) {
        steering_value = max_control;
    } else if (steering_value <= min_control) {
        steering_value = min_control;
    }

    CONSOLE_LOG(LOG_TAG, "%i, [%i, %i], [%i, %i]", steering_value, sense_l, sense_r, left_reading, right_reading);


    set_raw_steering(steering_value);
}

// void PID( int32_t kp, int32_t ki, int32_t kd) {
//     error = 0;
//     diff = false; 
//     //if (millis()- last_time > time_ms) {
//         diff = 0;
//         sense_r = digitalRead(TAPE_R);
//         sense_l = digitalRead(TAPE_L);
//         sense_rr = digitalRead(TAPE_RR);
//         sense_ll = digitalRead(TAPE_LL);
        
//         //0110
//         if( !sense_ll && sense_l && sense_r && !sense_rr){
//             error = error_table[4];
//         //1110
//         } else if( sense_ll && sense_l && sense_r && !sense_rr){
//             error = error_table[3];
//         //0111
//         }else if( !sense_ll && sense_l && sense_r && sense_rr){
//             error = error_table[5];
//         //0011
//         } else if( !sense_ll && !sense_l && sense_r && sense_rr){
//             error = error_table[6];
//         } 
//         //1100
//          else if( sense_ll && sense_l && !sense_r && !sense_rr){
//             error = error_table[2];
//         //1000
//         } else if( sense_ll && !sense_l && !sense_r && !sense_rr){
//             error = error_table[1];
//         //0001
//         } else if( !sense_ll && !sense_l && !sense_r && sense_rr){
//             error = error_table[7];
//         //0000
//         } else if( !sense_ll && !sense_l && !sense_r && !sense_rr){
//             if (last_error < 0) {
//                 error = error_table[8];
//                 // set_differential_steering(-20);
//                 diff = true;
//             } else if (last_error > 0){
//                 error = error_table[0];
//                 // set_differential_steering(20);
//                 diff =true; 
//             }
//         // } else if (sense_ll && sense_l && sense_r && sense_rr) {
//         //     error = 0;
//         } else {
//             error = last_error;
//         }
//         if (!diff) {
//             // set_differential_steering(0);
//         }
//         //CONSOLE_LOG(LOG_TAG,"%d,%d,%d,%d", sense_ll,sense_l, sense_r, sense_rr);
//         // CONSOLE_LOG(LOG_TAG,"%d",error);
//         total_error += error;
//         d_error = error-last_error;
//         control = kp*error + (ki*time_ms)*total_error + (kd/time_ms)*d_error + MID_POINT;
//         // CONSOLE_LOG(LOG_TAG,"%i",control);
//         if (control >= max_control) {
//             control = max_control;
//         } else if (control <= min_control) {
//             control = min_control;
//         }

//         if (abs(error) > 2) {
//             digitalWrite(PC13, HIGH);
//         } else {
//             digitalWrite(PC13, LOW);
//         }
//         // CONSOLE_LOG(LOG_TAG,"%i",control);
//         last_error = error;
//         last_r = sense_r;
//         last_l = sense_l;
//         // CONSOLE_LOG(LOG_TAG, "Time: %d",millis()-last_time);
//         last_time = millis();
//         // set_raw_steering(control);
//     //}
// }
bool tapeIsPresent() {
    return digitalRead(TAPE_L) || digitalRead(TAPE_R);
}

//servo center = 6% duty cycle 246
//servo max right = 10% duty cycle 490
//servo max left = 2% duty cycle 82

//steering bound 100-400








