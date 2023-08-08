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
uint32_t post_rocks_timer;
uint32_t lost_tape_timer;
uint32_t allCentralTapeSensors[] = {TAPE_L, TAPE_R};
uint32_t allOutsideTapeSensors[] = {TAPE_E_R};
uint32_t on_ramp_time;
extern int32_t kp;

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
        // CONSOLE_LOG(LOG_TAG, "roll: %i, pitch: %i, yaw: %i, on rocks: %d", getRoll(), getPitch(), getYaw(), isOnRocks());
        if (START_SIDE == HIGH) {
            // this line of code coule be very buggy, it is meant to say that stop once you have turned 90 degrees to the right. 
            if (getYaw() < -START_GYRO_CUTOFF) {
                centre_steering();
                // CONSOLE_LOG(LOG_TAG,"TURN COMPLETE");
                turnComplete = true;
            }
        } else {
            // same as above but now we are stopping once we have turned 90 degrees to the left. 
            if (getYaw() > START_GYRO_CUTOFF) {
                centre_steering();
                // CONSOLE_LOG(LOG_TAG,"TURN COMPLETE");
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





uint32_t on_rocks_timer;
uint32_t counter;
uint32_t have_seen_tape_once_timer;

StateMachine::state StateMachine::irState() {
    uint32_t timer = millis();
    // assigns the rock step be one at the start. this means that we have yet to reach the rocks. 
    if(!once){
        rock_step = 1;
        notOnRocksCounter = 0;
        onRocksCounter = 0;
        set_motor_speed(95);
        once = true;
        counter = 0;
    }

    // before rocks step

    // enter this block when we are on the rocks. 
    if (rock_step == 0) {
        //CONSOLE_LOG(LOG_TAG,"ROCK STEP 0");
        ir_PID();

        
        getPosition();
        // TODO: will need to add the back set of sensors later
        if (isOnRocks() && analogRead(TAPE_L) > BLACK_LEFT_CUTOFF && analogRead(TAPE_R) > BLACK_RIGHT_CUTOFF) {
            onRocksCounter++;
            // CONSOLE_LOG(LOG_TAG,"ON ROCKS ++");
        }
        // only continue to the next steps once multiple trials have been completed. 
        if (onRocksCounter >= NUMBER_OF_ROCKS_NEEDED) {
            // CONSOLE_LOG(LOG_TAG,"set rock state = 1");
            on_rocks_timer = millis();
            cut_motors();
            centre_steering();
            rock_step = 1;
        }
    }

    // on rocks step
    if (rock_step == 1) {

        if (millis() - on_rocks_timer > 300) {
            set_motor_speed(POST_ROCKS_MOTOR_SPEED);
            centre_steering();
        }
        // CONSOLE_LOG(LOG_TAG,"ROCK STEP 1");
        // ir_PID();

        if (analogRead(TAPE_L) < BLACK_LEFT_CUTOFF && analogRead(TAPE_R) < BLACK_RIGHT_CUTOFF) {
            set_motor_speed(POST_ROCKS_MOTOR_SPEED); 
            set_steering(POST_ROCKS_TURN_ANGLE);
            searching_for_tape_timer = millis();
            notOnRocksCounter++;
        } else {
            notOnRocksCounter = 0;
        }

        if (notOnRocksCounter == 2) {
            rock_step = 2;
            notOnRocksCounter = 0;
        }


        // getPosition();
        // if (isOnRocks) {
        //     notOnRocksCounter++;
        // } else {
        //     notOnRocksCounter = 0;
        // }
        
    //     // increment a consecetive not on rocks counter. 
    //     // if we have reached the set of consecituve not on rocks readings then we must enter the next state. 
        // if(notOnRocksCounter >= NUMBER_OF_NON_ROCKS_NEEDED) {
        //     // may need to be adjusted as needed.
        //     set_motor_speed(POST_ROCKS_MOTOR_SPEED); 
        //     set_steering(POST_ROCKS_TURN_ANGLE);
        //     searching_for_tape_timer = millis();
        //     rock_step = 2;
        // }
    }

    
    if(rock_step == 2) {
        // CONSOLE_LOG(LOG_TAG,"ROCK STEP 2");
        if (millis() - searching_for_tape_timer > RESTART_MOTORS_TIMER) {
            set_motor_speed(RESTART_MOTOR_SPEED);
            centre_steering();
            rock_step++;
        }
        
    }

    if (rock_step == 3) {
        // CONSOLE_LOG(LOG_TAG,"ROCK STEP 3");
        // if both sensors read black
        if (analogRead(TAPE_L) > BLACK_LEFT_CUTOFF && analogRead(TAPE_R) > BLACK_RIGHT_CUTOFF) {
            counter++;
        } 
        if (counter >= 3) {
            rock_step = 4;
            counter = 0;
        }

        //TODO: review if this is needed. 

        // if (millis() - searching_for_tape_timer > SEARCH_FOR_TAPE_TIME) {
        //    return TAPE_SEARCH;
        // }


    }

    


    if (rock_step == 4) {

        if (analogRead(TAPE_L) < BLACK_LEFT_CUTOFF && analogRead(TAPE_R) < BLACK_RIGHT_CUTOFF) {
            rock_step = 5;
        }

    }

    if (rock_step == 5) {
        spin_in_circle(false);
        set_steering(-100);
        rock_step = 6;
    }

    if (rock_step == 6) {
        if (analogRead(TAPE_L) > BLACK_LEFT_CUTOFF && analogRead(TAPE_R) > BLACK_RIGHT_CUTOFF) {
            have_seen_tape_once_timer = millis();
            rock_step = 7;
        }
    }

    

    if (rock_step == 7 && millis() - have_seen_tape_once_timer > 350) {
        if (analogRead(TAPE_L) < BLACK_LEFT_CUTOFF && analogRead(TAPE_R) < BLACK_RIGHT_CUTOFF) {
            counter++;
        }

        if (counter >= 15) {
            counter = 0;
            rock_step = 8;
        }
    }

    if (rock_step == 8) {
        set_motor_speed(55);
        set_steering(-78);
        rock_step = 9;
    }
    
    if(rock_step == 9 && (analogRead(TAPE_L) > BLACK_LEFT_CUTOFF || analogRead(TAPE_R) > BLACK_RIGHT_CUTOFF)) {
        once = false;
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
    
    return IR_FOLLOW; 
}

//currently is not used. Usefull in debbugging however. 
StateMachine::state StateMachine::tapeFollowState1() {
    analogPID(550,50,0);
    if(!once){
        resetTotal();
        set_motor_speed(95);
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
uint32_t average_incline_angle[6] = {0};
int incline_count = 0;

StateMachine::state StateMachine::tapeFollowState2() {

    if(!once) {
        resetTotal();
        follow_step = 0;
        set_motor_speed(55);
        //stores the position so that we can know when we are on the ramp. 
        storePosition();
        once = true;
        trial_counter_tape = 0;
        tape_follow_state_timer = millis();
    }
    if (follow_step < 1){
        digitalPID(KP,KI,KD);
    } else {
        digitalPID(15,0,0);
    }
    getPosition();
    double incline_angle;
    incline_angle = atan(sqrt(pow(tan((double) getPitch()* PI / 180.0), 2) + pow(tan( (double) getRoll()* PI / 180.0), 2))) * 180 / PI;
    if(incline_count >= sizeof(average_incline_angle)){
        incline_count = 0;
    }
    average_incline_angle[incline_count] = incline_angle;
    incline_count++;

    u_int16_t avg_incline = 0;

    for(int i = 0; i < 6; i++){
        avg_incline += average_incline_angle[i];
    }
    avg_incline = avg_incline / sizeof(average_incline_angle);
    //CONSOLE_LOG(LOG_TAG, "angle of incline: %i", (int) incline_angle);
    CONSOLE_LOG(LOG_TAG, "angle: %d", (int) avg_incline);


    // this block handles before you have hit the first marker. 
    if (follow_step == 0 && millis() - tape_follow_state_timer > 1000) {
        // CONSOLE_LOG(LOG_TAG, "pitch: %i", getPitch());

        if (avg_incline > 9) {
            trial_counter_tape++;
        } else {
            trial_counter_tape = 0;
        }
        
        //TODO: add checks for all central sensors
        if (trial_counter_tape >= 2) {
            trial_counter_tape = 0;
            
            // CONSOLE_LOG(LOG_TAG, "on ramp");
            set_motor_speed(82); //maybe needs extra error correction to make sure the others are still on the tape. 
            on_ramp_time = millis();
            follow_step = 1;
            // zeros the position 
        }
    }

    

    // this block handles before you have hit the second marker. 
    if (follow_step == 1 && millis()- on_ramp_time > 1500) {
        //TODO:ADD gyro check as well

        if ((avg_incline < 4)) {
            trial_counter_tape++;
        } else {
            trial_counter_tape = 0;
        }
        

        if (trial_counter_tape >= 4) {
            trial_counter_tape = 0;
            follow_step = 0;
            once = false;
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
        once = true;
    }

    current_jump_state = perform(current_jump_state);

    if (current_jump_state == isIRReady) {
        centre_steering();
        storePosition();
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

