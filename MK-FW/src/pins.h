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
// pin definitions
#define SERVO PA_2
#define LEFT_MOTOR_FORWARD PB_9
#define LEFT_MOTOR_BACKWARD PB_8
#define RIGHT_MOTOR_FORWARD PA_3
#define RIGHT_MOTOR_BACKWARD PA_6
#define I2C_SDA PB7
#define I2C_SCL PB6
#define ZERO PB4 
#define TAPE_L PA11
#define TAPE_LL PA12
#define TAPE_RR PB1
#define TAPE_R PB10
#define TAPE_E_L PB15
#define TAPE_E_R PB14
#define IR_R PA0
#define IR_L PA1
#define IR_E_R PA5
#define IR_E_L PA4
#define START_SIDE LOW
// Fixed variables 
#define LEFT_MAX 390
#define RIGHT_MAX 255
#define MID_POINT 320
#define IR_BEACON_DIST 100


