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


static const char* LOG_TAG = "MAIN";
extern StateMachine state_machine;

void setup() {
  delay(100);  // allow power to stabilize
  // if anything writes to these before started, it will crash.
  Serial.begin(9600);
  Serial.setTimeout(100);
  state_machine.init();
  pinMode(START_SIDE,INPUT);
  imuInit();
  pidInit();
  ir_init();
  pinMode(ZERO, INPUT);
  // pwm_start(MOTOR_2B, 1000, 2500, RESOLUTION_12B_COMPARE_FORMAT);
  // pwm_start(MOTOR_1B, 1000, 2500, RESOLUTION_12B_COMPARE_FORMAT);
}



void loop() {
  //state_machine.determineState();
  
  //pwm_start(SERVO, 50, PID(), RESOLUTION_12B_COMPARE_FORMAT);
  uint32_t high = LEFT_MAX;
  uint32_t low = RIGHT_MAX;
  for (int i = high ; i > low; i-=2) {
    pwm_start(SERVO, 50, i, RESOLUTION_12B_COMPARE_FORMAT);
    CONSOLE_LOG(LOG_TAG, "servo:%i",i);
    delay(30);
  }
  delay(500);
  
}
