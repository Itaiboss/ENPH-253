/**
 * @file state_machine.h
 * @brief StateMachine class header. Determines current state and switches between states
 */

#include <stdint.h>
#include <string>

#define START_HARDCODE_LENGTH 2000
#define LOST_MODE_OSCILLATION_TIME 1000;

class StateMachine {
 public:
    /**
     * @brief Possible device states
     */
    enum state {
        UNKNOWN                 =   0,
        START                   =   1,
        TAPE_FOLLOW_1            = 100,
        TAPE_FOLLOW_2            = 101,
        IR_FOLLOW               = 103,
        JUMP                    = 104,
        POST_JUMP               = 105,
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
    std::string getStateString(StateMachine::state);

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
    state postJumpState();
    state jumpState();
    private:
    // Previous, current and next state trackers
    volatile state prev_state;
    volatile state curr_state;
    volatile state next_state;

};