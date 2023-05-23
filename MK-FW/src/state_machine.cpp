/**
 * @file state_machine.h
 * @brief StateMachine class implementation. Determines current state and switches between states
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */
#include <state_machine.h>
#include <logs.h>
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
    CONSOLE_LOG(LOG_TAG, "Initialized the state machine");
}

state StateMachine::getCurrentState() {
    return curr_state;
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
        case ERROR
            next_state = errorState();
            break;
        case UNKNOWN:
        case INIT:
        default:
            next_state = initState();
            break;
    }

    if (curr_state != next_state) {
        CONSOLE_LOG(LOG_TAG, "Changing states");
        if (prev_state != curr_state) {
            prev_state = curr_state;
        }
        curr_state = next_state;
    }
}

tapeFollowState() {

}
irState() {

}
errorState() {

}
initState() {
    
}