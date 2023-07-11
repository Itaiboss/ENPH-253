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
#include <Adafruit_SSD1306.h>
#include <state_machine.h>
#include <logs.h>
#include <pid.h>

#pragma once

static const char* LOG_TAG = "MAIN";

#define SERVO PB_0

void setup() {
  StateMachine state_machine;
  delay(100);  // allow power to stabilize

  // if anything writes to these before started, it will crash.
  Serial.begin(9600);
  Serial.setTimeout(50);
  Wire.begin();
  state_machine.init();
  Serial.println("Starting");
}

void loop() {
  delay(500);
  //CONSOLE_LOG(LOG_TAG, "%d", PID());
  //pwm_start(SERVO, 50, PID(), RESOLUTION_12B_COMPARE_FORMAT);
}
