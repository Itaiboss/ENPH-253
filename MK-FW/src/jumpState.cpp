#include <jumpState.h>
#include <pins.h>
#include <control.h>
#include <imu.h>
#include <logs.h>
#include <math.h>
#include <ir.h>


static const char* LOG_TAG = "JUMP";



uint32_t trialCounter = 0;
uint32_t tape_checking_sensors[] =  {
    TAPE_R, 
    TAPE_E_R, 
    TAPE_L
};
uint32_t jump_checking_sensors[] = {
    TAPE_E_L, 
    TAPE_E_R, 
    TAPE_L, 
    TAPE_R
};

bool findSonar = false;
bool once_jump = false;


JumpState preform(JumpState current_state) {
    // No matter your state the trial counter will be reset to 0. 
    if(!once_jump) {
        trialCounter = 0;
    }

    // ONTAPE block. 
    if (current_state == onTape) {
        // angle the steering ever so slightly towards the left and kick the motors close to top speed. 
         
        if (!once_jump) {
            set_motor_speed(PREJUMP_MOTOR_SPEED);
            set_steering(PREJUMP_STEERING_ANGLE);
            // do note that we set the position in this state and not in the previous ones. (it may be redundent but it is here just in case we make a whoopsies)
            storePosition();
            once_jump = true;
        }
        
        // check the rightmost 3 sensor to see if the are reading white. 
        // To know we are off the tape we will have to have multiple consecutive trials where the tape goes white. 
        if (is_all_sensors_low(tape_checking_sensors, 3)) {
            trialCounter++;
        } else {
            trialCounter = 0;
        } 
        
        if (trialCounter >= NUM_OF_TRIALS) {
            once_jump = false;
            return offTape;
        }

        return onTape;
        

    }

    // OFFTAPE block

    if (current_state == offTape) {
        
        // checks to see if all 4 sensors are reading black. 
        // If they are all black for multiple consective times then we know we are now in the air. 
        if (is_all_sensors_high(jump_checking_sensors, 4)) {
            trialCounter++;
        } else {
            trialCounter = 0;
        }


        // once we have reached the maxmimum number of trials we will now move states. 
        if (trialCounter >= NUM_OF_TRIALS) {
            once_jump = false; 
            return inAir;
        }
    }
    

    // IN AIR block

    if (current_state == inAir) {
        // once we are in the air we should cut the motors to zero and turn our wheel in preperation of landing. 
        if (!once_jump) {
            CONSOLE_LOG(LOG_TAG, "cutting motors during jump");
            cut_motors();
            set_steering(LANDED_TURNING_ANGLE);
            once_jump = true;
        }
        
        // if we have multiple (non consecective) upwards acceleration we can move on to the onGroundState
        if (isUpwardsAcceleration()) {
            trialCounter++;
        }

        if (trialCounter >= NUM_OF_TRIALS) {
            once_jump = false;
            return onGround;
        }

        return inAir;
    }

    // ONGROUND block

    uint32_t on_ground_timer = 0;
    uint32_t on_ground_stepper = 0;
    bool has_centered_steering = false;

    if (current_state == onGround) {
        
        // start the given timer and assign the  step variable to 0.
        if (!once_jump) {
            on_ground_timer = millis();
            once_jump = true;
            has_centered_steering = false;
            on_ground_stepper = 0;
        }

        // STEP 0.

        // We start our engines and engage our diff once the timer has waited long enough. 
        // Note that steering is not set here as it has already been defined above. 

        if (millis() - on_ground_timer > ON_GROUND_WAIT_TIME && on_ground_stepper == 0) {
            set_motor_speed(LANDED_MOTOR_SPEED);
            set_differential_steering(LANDED_DIFFERENTIAL);
            on_ground_stepper = 1;
        }

        // STEP 1. 

        // We wait until we are off the rocks to continue. 
        // this block will validate that we are seeing the white part of the surface and not the rocks. 
        // (Again we wait until we have received enough consecuitive reading to move on)
        if (on_ground_stepper == 1) {
            if (is_all_sensors_low(jump_checking_sensors, 4)) {
                trialCounter++;
            } else {
                trialCounter = 0;
            }

            if (trialCounter >= NUM_OF_TRIALS) {
                on_ground_stepper = 2;
                trialCounter = 0;
            }
        }

        // STEP 2. 

        // This step is extremely experimental and will need to modified. 
        // basically we need to figure out the settings that get us close enough to the center of the board so that IR can be triggered at the correct time and the fastest possible. 

        uint32_t off_rocks_timer = 0;
        // option 1 (very traditional)
        if (on_ground_stepper == 2) {
            set_motor_speed(LOST_BOY_MOTOR_SPEED);
            set_steering(LOST_BOY_TURNING_ANGLE);
            set_differential_steering(LOST_BOY_DIFFERENTIAL);
            off_rocks_timer = millis();
            on_ground_stepper = 3;
        }

        // option 2 use the getDistacneToWall to guide our pathway. 

        // STEP 3. 

        
        // handles when we are in ideal position to collect IR beams. 
        if (on_ground_stepper == 3) {
            getPosition();
            if (getYaw() > MIN_ANGLE_TO_STOP_TURNING || getYaw() < -140) {
                // this is when we would expect us to reach this angle. and we shouyld stop our steering and continue our current speed forward. 
                if (on_ground_stepper == 3) {

                    centre_steering();
                    set_motor_speed(LOST_BOY_MOTOR_SPEED);
                    on_ground_stepper = 4;


                } else {
                    CONSOLE_LOG(LOG_TAG, "CRITICAL ERROR: we are turned around and probably still on the rocks. ");
                    return isLost;

                }


            }
        }

        // STEP 4. 

        // this step will handle the end condition if we go out turning range and return us to the spot we would like to be. 
        if(on_ground_stepper == 4) {
            getPosition();
            if (getYaw() < MIN_ANGLE_TO_STOP_TURNING) {
                set_differential_steering(30);
            } else if (getYaw() > -140) {
                set_differential_steering(-30);
            }
            on_ground_stepper = 3;
        }

        // overarching commands to get us in the correct modes once things are done in this state


        if (millis() - off_rocks_timer > WAITING_TIME) {
           cut_motors();
           return isLost;
        }

        if (on_ground_stepper >= 2) {
            if(IR_present()) {
                return isIRReady;
            }

        }

        return onGround;

    }

    CONSOLE_LOG(LOG_TAG, "CODE BUG (BLAME LOGAN): line should never be reached");
}


/**
 * @brief  Gets the expected distance to the wall based on the provideed yaw and sonar distances. 
 * @note   This function assumes you have recently exited the rocks. 
 * @param  sonarDistance: the distance to the wall provided by the sonar device in cm. 
 * @param  yawAngle: the current yaw Angle where 0 is towards the start position side of the board.
 * @retval the distance in centermetres to the wall on the other side. returns 0 if the wall is not present or the angle is less than 0 or greater than 180. 
 */
uint32_t getDistanceToWall(uint32_t sonarDistance, uint32_t yawAngle) {
    // with these given values you should not be able to see anything. 
    if (yawAngle < 0 || yawAngle > 180) {
        return 0;
    }

    return cos((double) yawAngle * PI / 180) * sonarDistance;

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


