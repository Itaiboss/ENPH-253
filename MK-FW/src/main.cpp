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
extern StateMachine state_machine;
JumpState jumpState = onTape;


void setup() {
  pinMode(PC13,OUTPUT);
  digitalWrite(PC13, LOW);
  delay(100);  // allow power to stabilize
  // if anything writes to these before started, it will crash.
  Serial.begin(9600);
  Serial.setTimeout(100);
  state_machine.init();
  pinMode(START_SIDE,INPUT);
  
  pidInit();
  ir_init();
  pinMode(ZERO, INPUT);
  pinMode(MOTOR_1A, OUTPUT);
  pinMode(MOTOR_1B, OUTPUT);//breaks imu
  pinMode(MOTOR_2A, OUTPUT);
  pinMode(MOTOR_2B, OUTPUT);
  imuInit();
  // set_motor_speed(0.6, true);
  set_steering(0, true);
  storePosition();
}

bool once_loop = false;

void loop() {
  //PID();

  //state_machine.determineState();
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



  // jumpState = preform(jumpState);


  
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

  state_machine.determineState();










  
  
}
