#include <jumpState.h>
#include <pins.h>
#include <control.h>
#include <imu.h>

uint32_t trialCounter = 0;
uint32_t tape_checking_sensors[] =  {TAPE_R, TAPE_E_R, TAPE_L};
uint32_t jump_checking_sensors[] = {TAPE_E_L, TAPE_E_R, TAPE_L, TAPE_R};

bool findSonar = false;


JumpState preform(JumpState current_state) {
    if (current_state == onTape) {
        
            set_motor_speed(0.9, true);
            set_steering(0.1, false);
            
        
    }
    

    if (current_state == inAir) {
        
            set_motor_speed(0, true);
            set_steering(0.7, false);
    }

    return get_next_state(current_state);

}

JumpState get_next_state(JumpState current_state) {
    if (current_state == onTape) {
        if (is_all_sensors_low(tape_checking_sensors, 3)) {
            trialCounter++;
        } else {
            trialCounter = 0;
        }

        if (trialCounter == NUM_OF_TRIALS) {
            trialCounter = 0;
            return offTape;
        }

        return onTape;
    }

    if (current_state == offTape) {
        if(is_all_sensors_high(jump_checking_sensors, 4)) {
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
        if (!digitalRead(sensor_names[i])) {
            return false;
        }
    }

    return true;
}

bool is_all_sensors_high(uint32_t sensor_names[], uint32_t size) {
    for(int i = 0; i < size; i++) {
        if (digitalRead(sensor_names[i])) {
            return false;
        }
    }

    return true;
}


