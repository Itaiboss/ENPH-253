#include <stdint.h>
#include <Wire.h>

#pragma once

// A control to manage the number of trials before we move on states. 
#define NUM_OF_TRIALS 5

// a set of controls to manage the prejump seqeunce
#define PREJUMP_STEERING_ANGLE 34
#define PREJUMP_MOTOR_SPEED 96

// a set of controls for the on rocks landed sequence. 
#define LANDED_TURNING_ANGLE 45
#define LANDED_MOTOR_SPEED 100
#define LANDED_DIFFERENTIAL 15
#define ON_GROUND_WAIT_TIME 500 // in milliseconds

// a set of controls for the post rocks landed sequence
#define LOST_BOY_TURNING_ANGLE 56
#define LOST_BOY_MOTOR_SPEED 65
#define LOST_BOY_DIFFERENTIAL 45
#define MIN_TURN_ANGLE 160;
#define WAITING_TIME 4500 // in milliseconds how long to wait before going into lost mode. 
#define MIN_ANGLE_TO_STOP_TURNING 170



#define MIN_SONAR_DIST 65 // minimum distance from the tape to the wall 

enum JumpState { onTape, offTape, inAir, onGround, isLost, isIRReady };

JumpState perform(JumpState current_state);

JumpState get_next_state(JumpState current_state);

bool is_all_sensors_low(uint32_t sensor_names[], uint32_t size);
bool is_all_sensors_high(uint32_t sensor_names[], uint32_t size);


