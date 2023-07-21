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
#pragma once

static const char* LOG_TAG = "STATE_MACHINE";

StateMachine state_machine;

StateMachine::StateMachine() {
}

StateMachine::~StateMachine() {
    CONSOLE_LOG(LOG_TAG, "Destroyed the state machine");
    }

void StateMachine::init() {
    prev_state = UNKNOWN;
    curr_state = INIT;
    next_state = UNKNOWN;
    CONSOLE_LOG(LOG_TAG, "also Initialized the state machine");
}

StateMachine::state StateMachine::getCurrentState() {
    return curr_state;
}

std::string StateMachine::getStateString(StateMachine::state) {
    std::string state;
    switch (state_machine.getCurrentState()) {
        case TAPE_FOLLOW:
            state = "TAPE_FOLLOW";
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
        case INIT:
            state = "INIT";
            break;
    }
    return state;
}

void StateMachine::determineState() {
    next_state = UNKNOWN;
    switch (curr_state) {
        case TAPE_FOLLOW:
            next_state = tapeFollowState();
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
        case START:
            next_state = startState();
            break;
        case UNKNOWN:
        case INIT:
        default:
            next_state = initState();
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
StateMachine::state initState() {

}
StateMachine::state startState() {

}

StateMachine::state irState() {

}

StateMachine::state tapeFollowState() {
    pwm_start(SERVO, 50, PID(), RESOLUTION_12B_COMPARE_FORMAT);

}

StateMachine::state jumpState() {

}

StateMachine::state errorState() {

}
