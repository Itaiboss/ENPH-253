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
  pidInit();
  ir_init();
}

void loop() {
<<<<<<< Updated upstream
  pwm_start(MOTOR_A, 1000, 2000, RESOLUTION_12B_COMPARE_FORMAT);
=======
  state_machine.determineState();
>>>>>>> Stashed changes
  // samples[i] = micros()-last;
  // last = micros();
  // i++;
  // if (i >=1000 && num<4){ 
  //   for (int j = 0 ; j< 1000 ; j++) {
  //     CONSOLE_LOG(LOG_TAG,"%i",samples[j]);
  //   }
  //   i=0;
  //   num++;
  // }
  //PID();
  // for (int i = 0; i <= 128; i++) { // goes from 0 degrees to 180 degrees
  //   // in steps of 1 degree
  //   digi_pot.writeWiper(i);              // tell servo to go to position in variable 'pos'
  //   ir_sample();
  //   delay(15);                       // waits 15ms for the servo to reach the position
  // }
  // for (int i = 128; i >= 0; i--) { // goes from 180 degrees to 0 degrees              // tell servo to go to position in variable 'pos'
  //   digi_pot.writeWiper(i);
  //   ir_sample();
  //   delay(15);                     // waits 15ms for the servo to reach the position
  // }
<<<<<<< Updated upstream
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
=======
  // uint32_t high = 400;
  // uint32_t low = 100;
  // for (int i = low ; i < high; i+=1) {
  //   pwm_start(SERVO, 50, i, RESOLUTION_12B_COMPARE_FORMAT);
  // CONSOLE_LOG(LOG_TAG, "servo:%i",270);
  //   delay(15);
  // }
  // delay(1000);
  // for (int i = high ; i > low; i-=1) {
  //   pwm_start(SERVO, 50, i, RESOLUTION_12B_COMPARE_FORMAT);
  //   CONSOLE_LOG(LOG_TAG, "servo:%i",i);
  //   delay(15);
  // }
  // delay(1000);
  
  // //pwm_start(MOTOR_A, 1000, (abs((int32_t)(steer-270))+100)/100*3000, RESOLUTION_12B_COMPARE_FORMAT);
   //pwm_start(MOTOR_A, 1000,2500, RESOLUTION_12B_COMPARE_FORMAT);
}
>>>>>>> Stashed changes
