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
bool side = false;
bool begin = false; 


void setup() {
  pinMode(PC13,OUTPUT);
  digitalWrite(PC13, LOW);
  delay(100);  // allow power to stabilize
  // if anything writes to these before started, it will crash.
  Serial.begin(9600);
  Serial.setTimeout(100);
  state_machine.init();
  pinMode(START_SIDE, INPUT_PULLUP);
  pinMode(BEGIN, INPUT_PULLUP);
  pidInit();
  ir_init();
  pinMode(ZERO, INPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  // imuInit();
  pinMode(LED, OUTPUT);
  // set_motor_speed(85);
  // pwm_start(LEFT_MOTOR_FORWARD, 1000, 4098, RESOLUTION_12B_COMPARE_FORMAT);
  // pwm_start(RIGHT_MOTOR_FORWARD, 1000, 3000, RESOLUTION_12B_COMPARE_FORMAT);
  
  start_time_main = millis();
  
  jumpState = onTape;
  begin = false;
}




bool once_main = false;
uint32_t speed_array[10];
uint32_t speed_index = 0;



void loop() {

  if(digitalRead(BEGIN) == LOW) {
    begin = true; 
  }
  if (begin){
    
    state_machine.determineState();
    // CONSOLE_LOG(LOG_TAG, "[%i, %i, %i, %i]", analogRead(IR_E_L), analogRead(IR_L), analogRead(IR_R), analogRead(IR_E_R));
  }
}
