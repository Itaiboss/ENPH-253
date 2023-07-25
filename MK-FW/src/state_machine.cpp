/**
 * @file state_machine.h
 * @brief StateMachine class implementation. Determines current state and switches between states
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */
#include <state_machine.h>
#include <arduino.h>
#include <logs.h>
#include <pins.h>
#include <stdint.h>
#include <pid.h>
#include <imu.h>
#include <sonar.h>
#pragma once

static const char* LOG_TAG = "STATE_MACHINE";

bool once = false;

StateMachine state_machine;
uint32_t start_time;
uint32_t rock_step;

StateMachine::StateMachine() {
}

StateMachine::~StateMachine() {
    CONSOLE_LOG(LOG_TAG, "Destroyed the state machine");
    }

void StateMachine::init() {
    prev_state = UNKNOWN;
    curr_state = START;
    next_state = UNKNOWN;
    CONSOLE_LOG(LOG_TAG, "Initialized the state machine");
}

StateMachine::state StateMachine::getCurrentState() {
    return curr_state;
}

std::string StateMachine::getStateString(StateMachine::state) {
    std::string state;
    switch (state_machine.getCurrentState()) {
        case TAPE_FOLLOW_1:
            state = "TAPE_FOLLOW_1";
            break;
        case TAPE_FOLLOW_2:
            state = "TAPE_FOLLOW_2";
            break;
        case IR_FOLLOW:
            state = "IR_FOLLOW";
            break;
        case ERROR:
            state = "ERROR";
            break;
        case START:
            state = "START";
            break;
        case JUMP:
            state = "JUMP";
            break;
        case UNKNOWN:
            state = "UNKNOWN";
            break;
    }
    return state;
}

void StateMachine::determineState() {
    next_state = UNKNOWN;
    switch (curr_state) {
        case START:
            next_state = startState();
            break;
        case TAPE_FOLLOW_1:
            next_state = tapeFollowState1();
            break;
        case TAPE_FOLLOW_2:
            next_state = tapeFollowState2();
            break;
        case IR_FOLLOW:
            next_state = irState();
            break;
        case ERROR:
            next_state = errorState();
            break;
        case JUMP:
            next_state = jumpState();
            break;
        case UNKNOWN:
        default:
            next_state = startState();
            break;
    }
    if (curr_state != next_state) {
        CONSOLE_LOG(LOG_TAG, "Moving from %s ----> %s", getStateString(curr_state), getStateString(next_state));
        if (prev_state != curr_state) {
            prev_state = curr_state;
        }
        curr_state = next_state;
    }
}
// StateMachine::state turnState() {

// }
StateMachine::state StateMachine::startState() {
    // if (!once) {
    //     if ( START_SIDE == HIGH ) {
    //         pwm_start(SERVO, 50, LEFT_MAX, RESOLUTION_12B_COMPARE_FORMAT);
    //     } else {
    //         pwm_start(SERVO, 50, RIGHT_MAX, RESOLUTION_12B_COMPARE_FORMAT);
    //     }
    //     start_time = millis();
    // } 
    // if (IR_DETECT) {
    //     once = false; 
    //     return IR_FOLLOW;
    // } else if (TAPE_DETECT || millis()- start_time > 2000) {
    //     once = false; 
    //     return TAPE_FOLLOW_1;
    // }
    // return START;
}

StateMachine::state StateMachine::irState() {
    // if(!detect_ir){
    //     return TAPE_FOLLOW_2;
    // }
    // if(!once){
    //     rock_step = 0;
    // }
    // pwm_start(SERVO, 50, ir_follow_steering_value(), RESOLUTION_12B_COMPARE_FORMAT);
    // bool on_rocks = isOnRocks();
    // if (on_rocks && rock_step == 0) {
    //     rock_step = 1;
    // }
    // if (!on_rocks && rock_step == 1) {
    //     if (getDistance() < IR_BEACON_DIST) {
    //         pwm_start(SERVO, 50, 300, RESOLUTION_12B_COMPARE_FORMAT);
    //         return TAPE_FOLLOW_2;
    //     }
    // }
    // return IR_FOLLOW; 
}

StateMachine::state StateMachine::tapeFollowState1() {
    pwm_start(SERVO, 50, PID(), RESOLUTION_12B_COMPARE_FORMAT);

}

StateMachine::state StateMachine::tapeFollowState2() {
    // pwm_start(SERVO, 50, PID(), RESOLUTION_12B_COMPARE_FORMAT);
    // if(ramp_tape == HIGH) {

    // }

}

StateMachine::state StateMachine::jumpState() {
    

}

StateMachine::state StateMachine::errorState() {

}
