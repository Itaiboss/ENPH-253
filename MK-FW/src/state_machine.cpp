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
uint32_t notOnRocksTapeSensors = 0;
uint32_t notOnRocksGyro = 0;
uint32_t onRocksCounter = 0;
uint32_t searching_for_tape_timer;
uint32_t post_rocks_timer;
uint32_t lost_tape_timer;
uint32_t allCentralTapeSensors[] = {TAPE_L, TAPE_R};
uint32_t allOutsideTapeSensors[] = {TAPE_E_R};
uint32_t on_ramp_time;
bool pos = false;
extern int32_t kp;
bool is_from_start = true;

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
    is_from_start = true;
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
        completed_turn_time = millis() ;
        once = true; 
    } 

    // this block will stop the turning once it reaches 90 degrees. 
    //if IR is present we send it to the ir follow state. 
    if (IR_present()) {
        ir_count++;
        if (ir_count > 3) {
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





uint32_t on_rocks_timer;
uint32_t counter;
uint32_t have_seen_tape_once_timer;

uint32_t lost_timer;

StateMachine::state StateMachine::irState() {
    CONSOLE_LOG(LOG_TAG, "step: %i", rock_step);
    uint32_t timer = millis();
    // assigns the rock step be one at the start. this means that we have yet to reach the rocks. 
    if(!once){
        rock_step = 0;
        notOnRocksTapeSensors = 0;
        notOnRocksGyro = 0;
        onRocksCounter = 0;
        set_motor_speed(95);
        storePosition();
        once = true;
        counter = 0;
        resetCounter();
        lost_timer = 0;
        digitalWrite(LED, LOW);
    }


    // before rocks step

    // enter this block when we are on the rocks. 
    if (rock_step == 0) {
        //CONSOLE_LOG(LOG_TAG,"ROCK STEP 0");
        ir_PID();


        // if (noIRFound()) {
        //     rock_step = 100;
        //     cut_motors();
        //     lost_timer = millis();
        // }

        // TODO: will need to add the back set of sensors later
        if (getIsClose()) {
            onRocksCounter++;
            // CONSOLE_LOG(LOG_TAG,"ON ROCKS ++");
        }
        // only continue to the next steps once multiple trials have been completed. 
        if (onRocksCounter >= 4) {
            // CONSOLE_LOG(LOG_TAG,"set rock state = 1");
            on_rocks_timer = millis();
            // digitalWrite(LED, HIGH);
            set_motor_speed(-100);       
            rock_step = 1;
        }
    }

    // on rocks step
    if (rock_step == 1) {

        if (millis() - on_rocks_timer > RESTART_MOTORS_TIMER) {
            set_motor_speed(POST_ROCKS_MOTOR_SPEED);
            set_steering(POST_ROCKS_TURN_ANGLE);
            if(analogRead(TAPE_L) < BLACK_LEFT_CUTOFF && analogRead(TAPE_R) < BLACK_RIGHT_CUTOFF) {
                rock_step = 2; 
            }
        }
    }


    if (rock_step == 2) {
        // CONSOLE_LOG(LOG_TAG,"ROCK STEP 3");
        // if both sensors read black
        if (analogRead(TAPE_L) > BLACK_LEFT_CUTOFF || analogRead(TAPE_R) > BLACK_RIGHT_CUTOFF) {
            counter++;
        } 
        if (counter >= 3) {
            rock_step = 3;
            counter = 0;
        }

        //TODO: review if this is needed. 

        // if (millis() - searching_for_tape_timer > SEARCH_FOR_TAPE_TIME) {
        //    return TAPE_SEARCH;
        // }


    }

    


    if (rock_step == 3) {

        if (analogRead(TAPE_L) < BLACK_LEFT_CUTOFF && analogRead(TAPE_R) < BLACK_RIGHT_CUTOFF) {
            rock_step = 4;
        }

    }

    if (rock_step == 4) {
        spin_in_circle(false);
        set_steering(-100);
        rock_step = 5;
    }

    if (rock_step == 5) {
        if (analogRead(TAPE_L) > BLACK_LEFT_CUTOFF && analogRead(TAPE_R) > BLACK_RIGHT_CUTOFF) {
            have_seen_tape_once_timer = millis();
            rock_step = 6;
        }
    }

    

    if (rock_step == 6 && millis() - have_seen_tape_once_timer > 0) {
        if (analogRead(TAPE_L) < BLACK_LEFT_CUTOFF || analogRead(TAPE_R) < BLACK_RIGHT_CUTOFF) {
            counter++;
        }

        if (counter >= 8) {
            counter = 0;
            rock_step = 7;
        }
    }

    if (rock_step == 7) {
        set_motor_speed(55);
        set_steering(-100);
        rock_step = 8;
    }
    
    if(rock_step == 8 && (analogRead(TAPE_L) > BLACK_LEFT_CUTOFF || analogRead(TAPE_R) > BLACK_RIGHT_CUTOFF)) {
        once = false;
        is_from_start = false;
        return TAPE_FOLLOW_2;
    }

    // if (rock_step >= 4) {
    //     // if (millis() - have_seen_tape_once_timer > PAST_TAPE_TIME_TO_GET_BACK_ON) {
    //     //     once = false;
    //     //     cut_motors();
    //     //     centre_steering();
    //     //     return TAPE_SEARCH;
    //     // }
    // }

    // if (rock_step == 100 && millis() - lost_timer >= 400) {
    //     is_from_start = false;
    //     return ERROR;
    // }
    
    return IR_FOLLOW; 
}

//currently is not used. Usefull in debbugging however. 
StateMachine::state StateMachine::tapeFollowState1() {
    analogPID(.6,0,0, 420, 380);
    if(!once){
        resetTotal();
        set_motor_speed(78);
        once = true;
    }
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
    isLeftActive = false;

    if (!once) {
        lost_tape_timer = millis();
        isRandomDirectionRight = false;
        current_turn_index = 0;

        if ((isRightActive && isLeftActive) || (!isRightActive && !isLeftActive)) {
            pwm_start(LEFT_MOTOR_FORWARD, 1000, isRandomDirectionRight ? 3200 : 0, RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(RIGHT_MOTOR_FORWARD, 1000, isRandomDirectionRight ? 0 : 3200, RESOLUTION_12B_COMPARE_FORMAT);
            current_turn_index = 0;
        }
        once = false;
        
    }

    // if only one of the sensors are active then we should try to max steer (on our own spot to find the tape).    
    if (isRightActive && !isLeftActive) {
        spin_in_circle(true);
        // sets the current index to what is happening 
        current_turn_index = 2;
        lost_tape_timer = millis();
    } else if (isLeftActive) {
        spin_in_circle(false);
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
        pwm_start(LEFT_MOTOR_FORWARD, 1000, isRandomDirectionRight ? 3200 : 0, RESOLUTION_12B_COMPARE_FORMAT);
            pwm_start(RIGHT_MOTOR_FORWARD, 1000, isRandomDirectionRight ? 0 : 3200, RESOLUTION_12B_COMPARE_FORMAT);
    }

    // will need to be changed when add the more sensors. 
    if (analogRead(TAPE_L) > BLACK_LEFT_CUTOFF || analogRead(TAPE_R) > BLACK_RIGHT_CUTOFF) {
        once = false;
        return TAPE_FOLLOW_2;
    }

    return TAPE_SEARCH;
}

int follow_step = 0;
uint32_t trial_counter_tape;
bool has_slowed_down = false;
uint32_t tape_follow_state_timer;
uint32_t average_incline_angle[6] = {0, 0, 0, 0, 0, 0};
int incline_count = 0;
uint32_t avg_incline = 0;
uint32_t time_array[100];
uint32_t index_tape_follow_2 = 0;

uint32_t gyro_reading_index = 0;

StateMachine::state StateMachine::tapeFollowState2() {
    

    if(!once) {
        resetTotal();
        follow_step = 0;
        set_motor_speed(69);
        //stores the position so that we can know when we are on the ramp. 
        once = true;
        trial_counter_tape = 0;
        tape_follow_state_timer = millis();
        // storePosition();
        pos = true; 
        index_tape_follow_2 = 0;
    }

    if (follow_step < 1) {
        analogPID(.6,0,0, 420, 380);
    } else {
        analogPID(.6,0,0, 420, 380);
    }

    if (follow_step == 0 && millis() - tape_follow_state_timer > 5000) {
        set_motor_speed(77);
        follow_step++;
    }    

    // this block handles before you have hit the second marker. 
    if (follow_step >= 0 && millis() - tape_follow_state_timer > 6000) {

        if (digitalRead(TAPE_E_L) && (analogRead(TAPE_L) > 200 || analogRead(TAPE_R) > 200)) {
            trial_counter_tape++;
        } else {
            trial_counter_tape = 0;
        }

        if (trial_counter_tape > 3) {
            trial_counter_tape = 0;
            follow_step = 0;
            once = false;
            current_jump_state = onTape;
            return JUMP;
        }
    }

    // time_array[index_tape_follow_2] = millis() - tape_follow_state_timer;

    // index_tape_follow_2++;
    // if (index_tape_follow_2 >= 100) {
    //     for (int i = 0; i < 100; i++) {
    //         CONSOLE_LOG(LOG_TAG, "%i", time_array[i]);
    //     }
    //     index_tape_follow_2 = 0;
    // }

    // gyro_reading_index++;

    return TAPE_FOLLOW_2;
}



StateMachine::state StateMachine::jumpState() {

    // if this if the first call the starting state should be onTape. 
    if(!once) {
        current_jump_state = onTape;
        once = true;
    }

    current_jump_state = perform(current_jump_state);

    if (current_jump_state == isIRReady) {
        centre_steering();
        once = false; 
        return IR_FOLLOW;
    }

    if (current_jump_state == isLost) {
        once = false;
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


    // IN this state we are looking to hookup with IR. 

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

