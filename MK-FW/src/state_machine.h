/**
 * @file state_machine.h
 * @brief StateMachine class header. Determines current state and switches between states
 */

#include <stdint.h>
#include <string>

// Settings for the start State
#define START_SPEED 75
#define START_TURNING_ANGLE 30
#define START_GYRO_CUTOFF 60
#define TIME_UNTIL_LOST_MODE 5000 // millis

// Modifications for the IR lost mode 
#define IR_LOST_MODE_OSCILLATION_TIME 1000 //millis

// Modification for the post rocks motor control 
#define POST_ROCKS_TURN_ANGLE 10
#define POST_ROCKS_MOTOR_SPEED 50

#define RESTART_MOTORS_TIMER 1000
#define RESTART_MOTOR_SPEED 30

// Settings for how the sensitive should be to the isOnRocks function. 
#define NUMBER_OF_NON_ROCKS_NEEDED 2
#define NUMBER_OF_ROCKS_NEEDED 4

// How long will search for tape inside the IR functiom
#define SEARCH_FOR_TAPE_TIME 2000 //millis

// How long should we turn in one direction when looking for tape near the IR beacon. 
#define TAPE_SEARCHING_MODE_MAX_ONE_DIRECTION_TURN 1000 // millis

class StateMachine {
 public:
    /**
     * @brief Possible device states
     */
    enum state {
        UNKNOWN                 =   0,
        START                   =   1,
        TAPE_FOLLOW_1           = 100,
        TAPE_FOLLOW_2           = 101,
        IR_FOLLOW               = 103,
        JUMP                    = 104,
        TAPE_SEARCH             = 105,
        ERROR                   = 900,
    };

    /**
     * @brief StateMachine object constructor
     */
    StateMachine();

    /**
     * @brief StateMachine object destructor
     */
    ~StateMachine();

    /**
     * @brief Initializes a new StateMachine object
     */
    void init();

    /**
     * @brief Get the current state of the device
     * 
     * @return current device state
     */
    state getCurrentState();

    /**
     * @brief Gets a state as a string
     * 
     * @return state as a string
     */
    std::string getStateString(StateMachine::state input);

    /**
     * @brief Determines the next state of the device
     * 
     */
    void determineState();

    state tapeFollowState1();
    state tapeFollowState2();
    state irState();
    state errorState();
    state initState();
    state startState();
    state jumpState();
    state tapeSearchState();
    private:
    // Previous, current and next state trackers
    volatile state prev_state;
    volatile state curr_state;
    volatile state next_state;

};