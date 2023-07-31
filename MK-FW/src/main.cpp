/**
 * @file      main.cpp
 * @brief     ENPH 253 competition main 
 *
 * @copyright Copyright ENPH253 (c) 2023
 *
 * @author Itai Boss
 * @author Logan Underwoof
 * @author Imogen neil
 * @author Brooklynn Erikson
 */
#include <stdint.h>
#include <Wire.h>
#include <state_machine.h>
#include <logs.h>
#include <pid.h>
#include <ir.h>
#include <pins.h>
#include <imu.h>
#include <control.h>
#include <jumpState.h>


static const char* LOG_TAG = "MAIN";
StateMachine state_machine;
JumpState jumpState = onTape;
uint32_t start_time_main = 0;
bool has_finished = false;


void setup() {
  pinMode(PC13,OUTPUT);
  digitalWrite(PC13, LOW);
  delay(100);  // allow power to stabilize
  // if anything writes to these before started, it will crash.
  Serial.begin(9600);
  Serial.setTimeout(100);
  state_machine.init();
  //pinMode(START_SIDE,INPUT);
  pidInit();
  ir_init();
  pinMode(ZERO, INPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  imuInit();
  storePosition();
  start_time_main = millis();

  /* 
  BLOCK 1. IR_PID
  */
  
  set_motor_speed(80);

  /*
  BLOCK 2. TAPE FOLLOW 
  */





 /*
 BLOCK 3. JUMP SEQUENCE. 
 */

// set_motor_speed(0.9, true);
// set_steering(0, false);


/*
BLOCK 4:  
*/

  // storePosition();
  // set_raw_steering(300);
  // set_motor_speed(0.65, true);
  // has_finished = false;


/*
BLOCK 5: more complicated jump sequence. 
*/
// jumpState = onTape;
}




// bool once= false;



void loop() {
  // set_raw_steering(MID_POINT);


  //PID();

  state_machine.determineState();
  //pwm_start(MOTOR_1A, 1000, 2000, RESOLUTION_12B_COMPARE_FORMAT);
  //pwm_start(MOTOR_2A, 1000, 2000, RESOLUTION_12B_COMPARE_FORMAT);
  
  // uint32_t high = LEFT_MAX;
  // uint32_t low = RIGHT_MAX;
  // for (int i = high ; i > low; i-=2) {
  //pwm_start(SERVO, 50, MID_POINT, RESOLUTION_12B_COMPARE_FORMAT);
  //   CONSOLE_LOG(LOG_TAG, "servo:%i",i);
  //   delay(30);
  // }
  // delay(500);

  // ir_PID();



  // jumpState = perform(jumpState);


  
  // CONSOLE_LOG(LOG_TAG, "long right: %i, right: %i, left: %i", digitalRead(TAPE_E_R), digitalRead(TAPE_R), digitalRead(TAPE_L));
  // CONSOLE_LOG(LOG_TAG, "current state: %i", jumpState);
  

  // if (jumpState == onGround) {
  //   if (!once_loop) {
  //     set_differential_steering(0.4, true);
  //     set_motor_speed(0.95, true);

  //   }
  //   getPosition();
  //   if (getYaw() > 160) {
  //     set_motor_speed(0, true);
  //   }

  // }

  // state_machine.determineState();

  

  
  /* 
  BLOCK 1. IR_PID
  */
  
  //ir_PID();

  /*
  BLOCK 2. TAPE FOLLOW 
  */

//  PID();

 /*
 BLOCK 3. JUMP SEQUENCE. 
 */
// if (once_loop) {
//   if (millis() - start_time_main > 4000) {
//     set_motor_speed(0, false);
//     set_raw_steering(300);
//     once_loop = false;
//   }
// }

/*
BLOCK 4:  
*/




// if(millis() - start_time_main > 900) {
//   set_motor_speed(0.7, true);
//   has_finished = true;
//   set_raw_steering(MID_POINT);
// }

// if(has_finished) {
//   ir_PID();
// }






/*
BLOCK 5: more complicated jump sequence. 
*/

// jumpState = perform(jumpState);

// if (jumpState == onGround) {
//   set_motor_speed(0.9, true);
//   set_raw_steering(300);
// }
// getPosition();
// CONSOLE_LOG(LOG_TAG, "roll: %i, pitch: %i, yaw: %i, on rocks: %d", getRoll(), getPitch(), getYaw(), isOnRocks());








  

    

  
}
