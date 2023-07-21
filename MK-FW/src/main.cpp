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
#include <MCP4131.h>
#include <SPI.h>



static const char* LOG_TAG = "MAIN";

#define SERVO PB_0
#define MOTOR_A PA_7
#define MOTOR_B PA_6
#define CLK PB3
#define MISO PB4
#define MOSI PB5
#define CS PA15
#define I2C_SDA PB7
#define I2C_SCL PB6


uint32_t last = 0;
uint32_t samples[1000];
uint32_t i = 0; 
uint32_t num = 0;
SPISettings SPI_settings(250000, MSBFIRST, SPI_MODE0);
MCP4131 digi_pot(CS);

void setup() {
  pinMode(CS, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CLK, OUTPUT);
  //Wire.begin(I2C_SDA,I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  //Wire.begin();
  //Wire.end();
  //Wire.endTransmission();
  //Wire.setClock(100000);
  SPI.setMISO(MISO);
  SPI.setMOSI(MOSI);
  SPI.setSCLK(CLK);
  StateMachine state_machine;
  delay(100);  // allow power to stabilize

  // if anything writes to these before started, it will crash.
  Serial.begin(9600);
  Serial.setTimeout(100);
  pidInit();
  state_machine.init();
  last = micros();
  ir_init();
  pwm_start(SERVO, 50, 10, RESOLUTION_12B_COMPARE_FORMAT);
}

void loop() {
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
  // uint32_t high = 340;
  // uint32_t low = 200;
  //pwm_start(MOTOR_A, 1000, 2200, RESOLUTION_12B_COMPARE_FORMAT);
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
  
  uint32_t turningCommand = ir_PID();
  CONSOLE_LOG(LOG_TAG, "command is %i", turningCommand);
  pwm_start(SERVO, 50, turningCommand, RESOLUTION_12B_COMPARE_FORMAT);
  
}
