#include <stdint.h>
#include <Wire.h>

#pragma once

#define NUM_OF_TRIALS 5
#define MIN_SONAR_DIST 65 // minimum distance from the tape to the wall 

enum JumpState { onTape, offTape, inAir, onGround };

JumpState preform(JumpState current_state);

JumpState get_next_state(JumpState current_state);

bool is_all_sensors_low(uint32_t sensor_names[], uint32_t size);
bool is_all_sensors_high(uint32_t sensor_names[], uint32_t size);


