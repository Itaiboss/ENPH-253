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
  // set_motor_speed(75);
  // set_differential_steering(50);
  centre_steering();
  // pwm_start(LEFT_MOTOR_FORWARD, 1000, 4098, RESOLUTION_12B_COMPARE_FORMAT);
  // pwm_start(RIGHT_MOTOR_FORWARD, 1000, 3000, RESOLUTION_12B_COMPARE_FORMAT);
  
  start_time_main = millis();
  
  jumpState = onTape;
}




bool once_main = false;



void loop() {

  // centre_steering();




  
  
  // set_raw_steering(25);
  // set_motor_speed( 85);

  // ir_PID();

  // CONSOLE_LOG(LOG_TAG, "[%i, %i]", analogRead(TAPE_L), analogRead(TAPE_R));
  // if (!once_main) {

    // if (millis() - start_time_main > 400) {
    //   spin_in_circle(false);
    //   set_steering(-100);
    //   once_main = true;
    // }
  // }



  // if (millis() - start_time_main > 2000) {
  //   cut_motors();
  //   centre_steering();
  // }








  // state_machine.determineState();

  //pwm_start(MOTOR_1A, 1000, 2000, RESOLUTION_12B_COMPARE_FORMAT);
  //pwm_start(MOTOR_2A, 1000, 2000, RESOLUTION_12B_COMPARE_FORMAT);
  
  // uint32_t high = LEFT_MAX;
  // uint32_t low = RIGHT_MAX;
  // for (int i = high ; i > low; i-=2) {
  //   pwm_start(SERVO, 50, i, RESOLUTION_12B_COMPARE_FORMAT);
  //   CONSOLE_LOG(LOG_TAG, "servo:%i",i);
  //   delay(30);
  // }
  // delay(500);

  // ir_PID();

  
  

  // state_machine.determineState();
}
