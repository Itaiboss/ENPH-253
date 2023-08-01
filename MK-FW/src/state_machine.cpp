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
#include <string.h>
#include <pid.h>
#include <imu.h>
#include <jumpState.h>
#include <ir.h>
#include <sonar.h>
#include <control.h>

#pragma once

static const char* LOG_TAG = "STATE_MACHINE";
bool once = false;

uint32_t start_time;
uint32_t rock_step;
uint32_t ir_count;
JumpState current_jump_state = onTape;
uint32_t notOnRocksCounter = 0;
uint32_t onRocksCounter = 0;
uint32_t searching_for_tape_timer;
uint32_t lost_tape_timer;
uint32_t allCentralTapeSensors[] = {TAPE_L, TAPE_R};
uint32_t allOutsideTapeSensors[] = {TAPE_E_L, TAPE_E_R};

bool direction_flip = true;


StateMachine::StateMachine() {
}

StateMachine::~StateMachine() {
    CONSOLE_LOG(LOG_TAG, "Destroyed the state machine");
    }

void StateMachine::init() {
    prev_state = UNKNOWN;
    curr_state = TAPE_FOLLOW_1;
    next_state = UNKNOWN;
    CONSOLE_LOG(LOG_TAG, "Initialized the state machine");
}

StateMachine::state StateMachine::getCurrentState() {
    return curr_state;
}

std::string StateMachine::getStateString(StateMachine::state input) {
    std::string state_string;
    switch (input) {
        case TAPE_FOLLOW_1:
            state_string = "TAPE_FOLLOW_1";
            break;
        case TAPE_SEARCH:
            state_string = "TAPE_SEARCH";
            break;
        case TAPE_FOLLOW_2:
            state_string = "TAPE_FOLLOW_2";
            break;
        case IR_FOLLOW:
            state_string = "IR_FOLLOW";
            break;
        case ERROR:
            state_string = "ERROR";
            break;
        case START:
            state_string = "START";
            break;
        case JUMP:
            state_string = "JUMP";
            break;
        case UNKNOWN:
            state_string = "UNKNOWN";
            break;
    }
    return state_string;
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
        case TAPE_SEARCH:
            next_state = tapeSearchState();
            break;
        case UNKNOWN:
            next_state = errorState();
            break;
        default:
            break;
    }

    if (curr_state != next_state) {
        CONSOLE_LOG(LOG_TAG, "Moving from %s ----> %s", getStateString(curr_state).c_str(), getStateString(next_state).c_str());
        if (prev_state != curr_state) {
            prev_state = curr_state;
        }
        curr_state = next_state;
        // reset the once here to double check that is has been set in the function. 
        once = false;
    }
}


bool turnComplete = false;
uint32_t completed_turn_time = 0;


StateMachine::state StateMachine::startState() {
    // on the first run it will set a hard turn to the right or left based on the starting position. HIGH means starting on the left side. 
    if (!once) {
        ir_count = 0;
        turnComplete = false;
        set_motor_speed(START_SPEED);
        // start on the left so we need a right turn
        if (START_SIDE == HIGH) {
            set_steering(-START_TURNING_ANGLE);
        } else {
            // START ON right
            set_steering(START_TURNING_ANGLE);
        }
        storePosition();
        completed_turn_time = millis() ;
        once = true; 
    } 

    // this block will stop the turning once it reaches 90 degrees. 

    if (!turnComplete) {
        getPosition();
        CONSOLE_LOG(LOG_TAG, "roll: %i, pitch: %i, yaw: %i, on rocks: %d", getRoll(), getPitch(), getYaw(), isOnRocks());
        if (START_SIDE == HIGH) {
            // this line of code coule be very buggy, it is meant to say that stop once you have turned 90 degrees to the right. 
            if (getYaw() < -START_GYRO_CUTOFF) {
                centre_steering();
                CONSOLE_LOG(LOG_TAG,"TURN COMPLETE");
                turnComplete = true;
            }
        } else {
            // same as above but now we are stopping once we have turned 90 degrees to the left. 
            if (getYaw() > START_GYRO_CUTOFF) {
                centre_steering();
                CONSOLE_LOG(LOG_TAG,"TURN COMPLETE");
                turnComplete = true;
            }
        }
    }
    //if IR is present we send it to the ir follow state. 
    if (IR_present()) {
        ir_count++;
        if (ir_count > 5) {
        once = false;
        ir_count = 0;
        centre_steering();
        return IR_FOLLOW;
        }
    } else {
        ir_count = 0;
    }

    if (millis() - completed_turn_time > TIME_UNTIL_LOST_MODE) {
        once = false;
        return ERROR;
    }
    //TODO: Investigate need for tape following if IR is not seen.

    return START;
}







StateMachine::state StateMachine::irState() {
    uint32_t timer = millis();
    // assigns the rock step be one at the start. this means that we have yet to reach the rocks. 
    if(!once){
        rock_step = 0;
        notOnRocksCounter = 0;
        onRocksCounter = 0;
        set_motor_speed(100);
        once = true;
        
    }

    // enter this block when we are on the rocks. 
    if (rock_step == 0) {
        ir_PID();
        getPosition();
        if (isOnRocks()) {
            onRocksCounter++;
            CONSOLE_LOG(LOG_TAG,"ON ROCKS ++");
        }
        // only continue to the next steps once multiple trials have been completed. 
        if (onRocksCounter >= NUMBER_OF_ROCKS_NEEDED) {
            CONSOLE_LOG(LOG_TAG,"set rock state = 1");
            rock_step = 1;
        }
    }

    if (rock_step == 1) {
        CONSOLE_LOG(LOG_TAG,"ROCK STEP 1");
        ir_PID();
        getPosition();
        if (isOnRocks) {
            notOnRocksCounter++;
        } else {
            notOnRocksCounter = 0;
        }
        // increment a consecetive not on rocks counter. 
        // if we have reached the set of consecituve not on rocks readings then we must enter the next state. 
        if(notOnRocksCounter >= NUMBER_OF_NON_ROCKS_NEEDED) {
            // may need to be adjusted as needed. 
            set_steering(POST_ROCKS_TURN_ANGLE);
            set_motor_speed(POST_ROCKS_MOTOR_SPEED);
            searching_for_tape_timer = millis();
            rock_step = 2;
            cut_motors();
        }
    }
    if (rock_step == 2) {
        CONSOLE_LOG(LOG_TAG,"ROCK STEP 2");
        //will need to be changed when add the more sensors. 
        if (!is_all_sensors_low(allCentralTapeSensors, 2)) {
            once = false;
            return TAPE_FOLLOW_2;
        }

        if (!is_all_sensors_low(allOutsideTapeSensors, 2)) {
            once = false;
            return TAPE_SEARCH;
        }

        if (millis() - searching_for_tape_timer > SEARCH_FOR_TAPE_TIME) {
            once = false; 
            return TAPE_SEARCH;
        }
    }
    CONSOLE_LOG(LOG_TAG,"Time:%d", millis()-timer);
    return IR_FOLLOW; 
}

//currently is not used. Usefull in debbugging however. 
StateMachine::state StateMachine::tapeFollowState1() {
    PID();
    return TAPE_FOLLOW_1;
}

bool isRightActive;
bool isLeftActive;
/* current_turn_index refers to 
* 0 -> random direction
* 1 -> left turn max
* 2 -> right turn max
*/
uint32_t current_turn_index;
bool isRandomDirectionRight;

// this needs a comprehensive review. It is really hard set of logic to follow. 
StateMachine::state StateMachine::tapeSearchState() {

    isRightActive = digitalRead(TAPE_E_R);
    isLeftActive = digitalRead(TAPE_E_L);

    if (!once) {
        lost_tape_timer = millis();
        isRandomDirectionRight = false;
        current_turn_index = 0;

        if ((isRightActive && isLeftActive) || (!isRightActive && !isLeftActive)) {
            set_motor_speed(75);
            set_differential_steering(isRandomDirectionRight ? -45 : 45);
            current_turn_index = 0;
        }
        once = false;
        
    }

    // if only one of the sensors are active then we should try to max steer (on our own spot to find the tape).    
    if (isRightActive && !isLeftActive) {
        // max turn to the right
        set_motor_speed(70);
        set_differential_steering(-100);
        // sets the current index to what is happening 
        current_turn_index = 2;
        lost_tape_timer = millis();
    } else if (isLeftActive) {
        set_motor_speed(70);
        set_differential_steering(100);
        // sets the current index to what is happening 
        current_turn_index = 1;
        lost_tape_timer = millis();
    }

    // if an amount of time 

    if (millis() - lost_tape_timer > TAPE_SEARCHING_MODE_MAX_ONE_DIRECTION_TURN) {
        switch(current_turn_index) {
        case 0:
            isRandomDirectionRight = isRandomDirectionRight ? false : true;
            break;
        case 1: 
        // we were jus turning left so we should try the other direction. 
            isRandomDirectionRight = true;
            break;
        case 2: 
        // we were jus turning right so we should try the other direction. 
            isRandomDirectionRight = false;
            break;
        }
        set_motor_speed(75);
        set_differential_steering(isRandomDirectionRight ? -45 : 45);
    }

    // will need to be changed when add the more sensors. 
    if (!is_all_sensors_low(allCentralTapeSensors, 2)) {
        once = false;
        return TAPE_FOLLOW_2;
    }

    return TAPE_SEARCH;
}

int follow_step = 0;

StateMachine::state StateMachine::tapeFollowState2() {

    if(!once) {
        follow_step = 0;
        storePosition();
    }
    PID();
    getPosition();
    // this block handles before you have hit the first marker. 
    if (follow_step == 0) {
        //TODO: add checks for all central sensors
        if (digitalRead(TAPE_E_L) && !digitalRead(TAPE_E_R)|| getPitch()>=15) { //maybe needs extra error correction to make sure the others are still on the tape. 
            follow_step = 1;
            // zeros the position 
        }
    } 
    // this block handles before you have hit the second marker. 
    if (follow_step == 1) {
        //TODO:ADD gyro check as well
        if (digitalRead(TAPE_E_L) && digitalRead(TAPE_E_R) && ((digitalRead(TAPE_L) || digitalRead(TAPE_R)))) {
            follow_step = 0;
            storePosition();
            return JUMP;
        }
    }
    return TAPE_FOLLOW_2;
}



StateMachine::state StateMachine::jumpState() {


    // if this if the first call the starting state should be onTape. 
    if(!once) {
        current_jump_state = onTape;
    }

    current_jump_state = perform(current_jump_state);

    if (current_jump_state == isIRReady) {
        return IR_FOLLOW;
    }

    if (current_jump_state == isLost) {
        return ERROR;
    }

    return JUMP;

}


StateMachine::state StateMachine::errorState() {

    // complete a spin in different direction trying to pick up the IR beacon. 
    // i don't think we should be looking for tape becuase that would completly fuck with our sequencing. We would be as good as lost at that point -Logan
    if(!once) {
        ir_count = 0;
        start_time = millis();
        set_motor_speed(70);
        once = true; 
        set_differential_steering(direction_flip ? 100 : -100);
    }

    if (IR_present()) {
        ir_count++;
        if (ir_count > 5) {
        once = false;
        ir_count = 0;
        return IR_FOLLOW;
        }
    } else {
        ir_count = 0;
    }

    if (millis()- IR_LOST_MODE_OSCILLATION_TIME >= start_time) {
        once = false; 
        direction_flip = direction_flip ? false : true;
        return ERROR;
    }

    return ERROR;
}
