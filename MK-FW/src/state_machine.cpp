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
uint32_t numOfConsececutiveNonRocks = 10;

StateMachine::StateMachine() {
}

StateMachine::~StateMachine() {
    CONSOLE_LOG(LOG_TAG, "Destroyed the state machine");
    }

void StateMachine::init() {
    prev_state = UNKNOWN;
    curr_state = TAPE_FOLLOW_2;
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


bool turnComplete = false;
uint32_t start_startTime = 0;


StateMachine::state StateMachine::startState() {
    // on the first run it will set a hard turn to the right or left based on the starting position. HIGH means starting on the left side. 
    if (!once) {
        startTime = millis();
        turnComplete = false;
        set_motor_speed(0.8, true);
        if (START_SIDE == HIGH) {
            set_steering(0, true);
        } else {
            set_steering(0, false);
        }
        start_time = millis();
        storePosition();
        once = true; 
    } 

    // this block will stop the turning once it reaches 90 degrees. 

    if (!turnComplete) {
        getPosition();
        if (START_SIDE == HIGH) {
            // this line of code coule be very buggy, it is meant to say that stop once you have turned 90 degrees to the right. 
            if (getYaw() < -80) {
                set_steering(0, true);
                turnComplete = true;
            }
        } else {
            // same as above but now we are stopping once we have turned 90 degrees to the left. 
            if (getYaw() > 80) {
                set_steering(0, true);
                turnComplete = true;
            }
        }
    }

    // if IR is present we send it to the ir follow state. 

    if (IR_present()) {
        once = false; 

        return IR_FOLLOW;
    } 

    if (millis() - start_startTime > START_HARDCODE_LENGTH) {
        CONSOLE_LOG(LOG_TAG, "Unknown state has been reached.");
        once = false;
        return ERROR;
    }

    return START;
}


uint32_t notOnRocksCounter = 0;
uint32_t onRocksCounter = 0;
uint32_t howRocksAreNeeded = 4;



StateMachine::state StateMachine::irState() {
   
    // assigns the rock step be one at the start. this means that we have yet to reach the rocks. 
    if(!once){
        rock_step = 0;
        notOnRocksCounter = 0;
        onRocksCounter = 0;
    }

    ir_PID();
    getPosition();
    bool on_rocks = isOnRocks();
    // enter this block when we are on the rocks. 
    if (on_rocks && rock_step == 0) {
        onRocksCounter++;
        // only continue to the next steps once multiple trials have been completed. 
        if (onRocksCounter == howRocksAreNeeded) {
            rock_step = 1;
        }
    }

    if (!on_rocks && rock_step == 1) {
        // increment a consecetive not on rocks counter. 
        notOnRocksCounter++;

        // if we have reached the set of consecituve not on rocks readings then we must enter the next state. 

        if(notOnRocksCounter == numOfConsececutiveNonRocks) {
            once = false;
            // may need to be adjusted as needed. 
            set_steering(0.2, false);
            return TAPE_FOLLOW_2;
        }
        // if (getDistance() < IR_BEACON_DIST) {
        //     set_steering(0.25);
        //     once = false;
        //     return TAPE_FOLLOW_2;
        // }

        // will need to add this in once sonar is up and running. not even sure if it is needed. 

        
    } else {
        notOnRocksCounter = 0;

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
    // if (follow_step == 0) {
    //     if (digitalRead(TAPE_E_R) && !digitalRead(TAPE_E_L)) { //maybe needs extra error correction to make sure the others are still on the tape. 
    //         follow_step = 1;
    //         // zeros the position 
    //         getPosition();
    //         storePosition();
    //     }
    //     return TAPE_FOLLOW_2;
    // } 

    // this block handles before you have hit the second marker. 
    if (follow_step == 1) {
        if (digitalRead(TAPE_E_R) && ((digitalRead(TAPE_L) || digitalRead(TAPE_R)))) {
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

uint32_t postJumpTimer = 0;

uint32_t isnt_on_rocks_counter = 0;
uint32_t hasCutMotors = false;

StateMachine::state StateMachine::postJumpState() {

    // we are assuming that we will not fall off the edge in the case of failure. 
    if (!once) {
        postJumpTimer = millis();
        getPosition();
        set_steering(350);
        set_motor_speed(0.85, true);
        hasFinishedTurning = false;
        once = true;
        isnt_on_rocks_counter = 0;
    }

    getPosition();

    if (!isOnRocks() && !hasCutMotors) {
        isnt_on_rocks_counter++;
        if (isnt_on_rocks_counter >= 5) {
            set_motor_speed(0.65, true);
            set_differential_steering(0.9, false);

            hasCutMotors = true;
        }
    } else {
        isnt_on_rocks_counter = 0;
    }

    

    
    if (IR_present()) {
        once = false; 
        return IR_FOLLOW;
    }
    if (!hasFinishedTurning) {
        
        if (getYaw() > 160) {
            set_steering(0, false);
            set_differential_steering(0, false);
            hasFinishedTurning = true;
        } 
    }

    if (millis() - postJumpTimer > 10000) {
        once = false;
        return ERROR;
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
        set_motor_speed(0.6, true);
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
