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
StateMachine state_machine;

void setup() {
  Wire.begin(I2C_SDA,I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  delay(100);  // allow power to stabilize
  // if anything writes to these before started, it will crash.
  Serial.begin(9600);
  Serial.setTimeout(100);
  state_machine.init();
  imuInit();
  pidInit();
  ir_init();
  pinMode(ZERO, INPUT);
}

void loop() {
  pwm_start(MOTOR_A, 1000, 2000, RESOLUTION_12B_COMPARE_FORMAT);
  // uint32_t high = 310;
  // uint32_t low = 240;
  // for (int i = low ; i < high; i+=1) {
  //   pwm_start(SERVO, 50, i, RESOLUTION_12B_COMPARE_FORMAT);
  //   CONSOLE_LOG(LOG_TAG, "servo:%i",i);
  //   delay(30);
  // }
  // for (int i = high ; i > low; i-=1) {
  //   pwm_start(SERVO, 50, i, RESOLUTION_12B_COMPARE_FORMAT);
  //   CONSOLE_LOG(LOG_TAG, "servo:%i",i);
  //   delay(30);
  // }
  pwm_start(SERVO, 50, PID(), RESOLUTION_12B_COMPARE_FORMAT);
}
