/**
 * @file state_machine.h
 * @brief StateMachine class header. Determines current state and switches between states
 */

#include <stdint.h>
#include <string>
class StateMachine {
 public:
    /**
     * @brief Possible device states
     */
    typedef enum {
        UNKNOWN                 =   0,
        INIT                    =   1,
        TAPE_FOLLOW             = 100,
        IR_FOLLOW               = 101,
        ERROR                   = 900,
    } state;

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
    std::string StateMachine::getStateString(StateMachine::state);

    /**
     * @brief Returns the next state of the device
     * 
     * @return next device state
     */
    state getNextState();

    /**
     * @brief Determines the next state of the device
     * 
     */
    void determineState();

 private:
    // Previous, current and next state trackers
    volatile state prev_state;
    volatile state curr_state;
    volatile state next_state;

};