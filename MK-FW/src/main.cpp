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
#include <imu.h>

#pragma once

static const char* LOG_TAG = "MAIN";
int testVal = 0;

#define SERVO PB_0
#define ZERO PB4

void setup() {
  StateMachine state_machine;
  delay(100);  // allow power to stabilize

  // if anything writes to these before started, it will crash.
  Serial.begin(9600);
  Serial.setTimeout(50);
  Wire.begin();
  state_machine.init();
  imuInit();
  Serial.println("Starting");
  pinMode(ZERO, INPUT);
}

void loop() {
  bool rock = isOnRocks();
  int start = millis();
  getPosition();
  CONSOLE_LOG(LOG_TAG, "Roll:%i, Pitch:%i, Yaw:%i ", getRoll(), getPitch(), getYaw());
  //CONSOLE_LOG(LOG_TAG, "%i",  millis() - start );
  //CONSOLE_LOG(LOG_TAG,"On rocks: %d",rock);
  //CONSOLE_LOG(LOG_TAG, "%d", PID());
  //pwm_start(SERVO, 50, PID(), RESOLUTION_12B_COMPARE_FORMAT);
}
