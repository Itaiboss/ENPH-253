#include <jumpState.h>
#include <pins.h>
#include <control.h>
#include <imu.h>
#include <logs.h>


static const char* LOG_TAG = "JUMP";



uint32_t trialCounter = 0;
uint32_t tape_checking_sensors[] =  {
    TAPE_R, 
    TAPE_E_R, 
    TAPE_L
};
uint32_t jump_checking_sensors[] = {
    // TAPE_E_L, 
    TAPE_E_R, 
    TAPE_L, 
    TAPE_R
};

bool findSonar = false;
bool once_jump = false;
JumpState preform(JumpState current_state) {
    if (current_state == onTape) {

        if (!once_jump) {
            set_motor_speed(0.9, true);
            set_steering(245);
            once_jump = true;
        }
            
    }
    

    if (current_state == inAir) {


        if (!once_jump) {
            set_motor_speed(0, true);
            set_steering(350);
            once_jump = true;
        }
    }

    JumpState nextState = get_next_state(current_state);
    if (nextState != current_state) {
        once_jump = false;
    }

    return nextState;

}

JumpState get_next_state(JumpState current_state) {
    if (current_state == onTape) {
        if (is_all_sensors_low(tape_checking_sensors, 3)) {
            trialCounter++;
            CONSOLE_LOG(LOG_TAG, "all sensors are low");
        } else {
            trialCounter = 0;
        }

        if (trialCounter == NUM_OF_TRIALS) {
            trialCounter = 0;
            CONSOLE_LOG(LOG_TAG, "move to next state");
            return offTape;
        }

        return onTape;
    }

    if (current_state == offTape) {
        if(is_all_sensors_high(jump_checking_sensors, 3)) {
            trialCounter++;
        } else {
            trialCounter = 0;
        }

        if (trialCounter == NUM_OF_TRIALS) {
            trialCounter = 0;
            return inAir;
        }

        return offTape;

    }

    if (current_state == inAir) {
        getPosition();
        if (isUpwardsAcceleration()) {
            trialCounter++;
        }

        if(trialCounter == NUM_OF_TRIALS) {
            trialCounter = 0;
            return onGround;
        }
        return inAir;

    }

    return onTape;
}


/**
 * @brief  Checks if the all the provided sensors are low. 
 * @note   
 * @param  sensor_names[]: a list of names for the sensors. 
 * @param  size: the size of the provided array. 
 * @retval 
 */

bool is_all_sensors_low(uint32_t sensor_names[], uint32_t size) {
    for(int i = 0; i < size; i++) {
        if (digitalRead(sensor_names[i])) {
            return false;
        }
    }
    return true;
}

bool is_all_sensors_high(uint32_t sensor_names[], uint32_t size) {
    for(int i = 0; i < size; i++) {
        if (!digitalRead(sensor_names[i])) {
            return false;
        }
    }

    return true;
}


