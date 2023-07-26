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
#include <jumpState.h>
#include <ir.h>
#include <sonar.h>
#include <control.h>
#pragma once

static const char* LOG_TAG = "STATE_MACHINE";
bool once = false;

StateMachine state_machine;
uint32_t start_time;
uint32_t rock_step;
JumpState current_jump_state = onTape;
uint32_t startTime = 0;
bool direction_flip = true;

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
        case POST_JUMP:
            next_state = postJumpState();
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

bool turnComplete = false;
uint32_t start_startTime = 0;


StateMachine::state StateMachine::startState() {
    // on the first run it will set a hard turn to the right or left based on the starting position. 
    if (!once) {
        startTime = millis();
        turnComplete = false;
        set_motor_speed(0.8, true);
        if (START_SIDE == HIGH) {
            set_steering(0);
        } else {
            set_steering(1);
        }
        start_time = millis();
        once = true; 
    } 

    // this block will stop the turning once it reaches 90 degrees. 

    if (!turnComplete) {
        if (getYaw() > 90) {
            set_steering(0.5);
            turnComplete = true;
        }
    }

    // if IR is present we send it to the ir follow state. 

    if (IR_present()) {
        once = false; 

        return IR_FOLLOW;
    } 

    if (millis() - start_startTime > START_HARDCODE_LENGTH) {
        CONSOLE_LOG(LOG_TAG, "Unknown state reached please fix");
        once = false;
        return ERROR;
    }

    return START;
}



StateMachine::state StateMachine::irState() {
   
    // assigns the rock step be one at the start. this means that we have yet to reach the rocks. 
    if(!once){
        rock_step = 0;
    }
    ir_PID();
    bool on_rocks = isOnRocks();
    if (on_rocks && rock_step == 0) {
        rock_step = 1;
    }

    if (!on_rocks && rock_step == 1) {
        // if (getDistance() < IR_BEACON_DIST) {
        //     set_steering(0.25);
        //     once = false;
        //     return TAPE_FOLLOW_2;
        // }

        // will need to add this in once sonar is up and running. 

        once = false;
        set_steering(0.35);
        return TAPE_FOLLOW_2;


    }

    return IR_FOLLOW; 
}

//currently is not used. 
StateMachine::state StateMachine::tapeFollowState1() {
    PID();
}

int follow_step = 0;

StateMachine::state StateMachine::tapeFollowState2() {
    PID();
    // this block handles before you have hit the first marker. 
    if (follow_step == 0) {
        if (digitalRead(TAPE_E_R) && !digitalRead(TAPE_E_L)) { //maybe needs extra error correction to make sure the others are still on the tape. 
            follow_step = 1;
            imuZero();
        }
        return TAPE_FOLLOW_2;
    } 

    // this block handles before you have hit the second marker. 
    if (follow_step == 1) {
        if (digitalRead(TAPE_E_L) && digitalRead(TAPE_E_R)) {
            follow_step = 0;
            current_jump_state = onTape;
            return JUMP;
        }
    }
    

}



StateMachine::state StateMachine::jumpState() {

    current_jump_state = preform(current_jump_state);

    if (current_jump_state == onGround) {
        current_jump_state = onTape;
        return POST_JUMP;
    } 
    
    return JUMP;

}

bool hasFinishedTurning = false;

StateMachine::state StateMachine::postJumpState() {

    // we are assuming that we will not fall of the edge in the case of failure. 
    if (!once) {
        set_steering(0);
        hasFinishedTurning = false;
        once = true;
    }
    if (IR_present()) {
        once = false; 
        return IR_FOLLOW;
    }
    if (!hasFinishedTurning) {
        if (getRoll() < 180) { // will need some fixing
            set_steering(0.5);
            hasFinishedTurning = true;
        } 
    }
}

    // bool is_on_rocks = isOnRocks();
    // getPosition();

    // if(findSonar && getDistance() < MIN_SONAR_DIST){
    //     //cut motor, slow down
    // }
    // else if(findSonar && getDistance() < minDistance){
    //     //turn LEFT, turn on tape sensors to find tape
    //     //tape sensor shouldn't actually turn on until tape is detected somewhere, then it'll effect the steering
    // }

    // //else if(!IRdetected)
    // {
    //     //for overshoot, going to fall off short edge of course
    //     if(!isPresent() && !rock){//front sonar
    //         //cut motor, reverse
    //     }
    //     //for angle too far right, going to fall off long edge by rocks
    //     else if(getYaw() > 10){
    //         //cut motor, reverse
    //     }
    //     else if(isPresent){
    //         //adjust turn to be slightly more centre
    //     }
    //     else{
    //         //turn LEFT
    //     }
    // }
    // //else if(IRdetected), ir follow state
    // {
    //     //turn to centre and go to beacon
    //     if(rock){
    //         findSonar = true; //turn on front sonar
    //     }

    // }


StateMachine::state StateMachine::errorState() {
    if(!once) {
        start_time = millis();
        once = true; 
        set_differential_steering(1, direction_flip);
    }

    if(IR_present()) {
        once = false;
        return IR_FOLLOW;
    }

    if (millis() > LOST_MODE_OSCILLATION_TIME + start_time) {
        once = false; 
        if (direction_flip) { 
            direction_flip = false;
        } else {
            direction_flip = true;
        }
        return ERROR;
    }

    return ERROR;



}
