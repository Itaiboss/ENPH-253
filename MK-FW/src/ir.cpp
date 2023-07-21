/**
 * @file ir.cpp
 * @brief contains methods for IR sensing and convolution
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */
#include <ir.h>
#include <logs.h>
#include <arduinoFFT.h>

static const char* LOG_TAG = "IR";

double left_ir_reading[NUM_SAMPLES] = {0};
double right_ir_reading[NUM_SAMPLES] = {0};
double imaginary[NUM_SAMPLES] = {0};
uint32_t valueHistory[ROLLING_AVERAGE_NUMBER] = {0};
uint32_t signalAmplitudeCutOff = 20;

// A one kilohertz sine wave.
uint32_t left_max_val = 0;
uint32_t left_min_val = 1024;
uint32_t right_max_val = 0;
uint32_t right_min_val = 1024;



// the following variables are used to track the 
// different prop, derivative, and integral error terms. 
int32_t current_error = 0;
int32_t last_error_IR = 0;
int32_t total_error_IR = 0;

// the following are coefficents to control the PID logic 
double prop_coef = 1;
double derivative_coef = 0;
double integral_coef = 0;

uint32_t max_left_turn = 200;
uint32_t max_right_turn = 400;
uint32_t middle = 300;







void ir_init() {
  pinMode(IR_READ_LEFT, INPUT);
  pinMode(IR_READ_RIGHT, INPUT);
  CONSOLE_LOG(LOG_TAG, "Initializing IR");
  // gives the default values for 
  for (int i = 0; i < ROLLING_AVERAGE_NUMBER; i++) {
    valueHistory[i] = middle;
  }
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType);

uint32_t ir_PID() {
  
  uint32_t timer;
  uint32_t time_marker = millis();
  uint32_t sample_time = 0;
  current_error = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    timer = micros();
    imaginary[i] = 0;
    left_ir_reading[i] = analogRead(IR_READ_LEFT);
    right_ir_reading[i] = analogRead(IR_READ_RIGHT);
    if(left_ir_reading[i] > left_max_val) {
      left_max_val = left_ir_reading[i];
    }
    if(left_ir_reading[i] < left_min_val) {
      left_min_val = left_ir_reading[i];
    }

    if(right_ir_reading[i] > right_max_val) {
      right_max_val = right_ir_reading[i];
    }
    if(right_ir_reading[i] < right_min_val) {
      right_min_val = right_ir_reading[i];
    }
    sample_time += micros() - timer;
  }
  sample_time = sample_time / NUM_SAMPLES;
  // gets the left frequency.
  arduinoFFT FFT = arduinoFFT(left_ir_reading, imaginary, NUM_SAMPLES, (double) 1/(sample_time*pow(10, -6)));
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  double left_frequency = FFT.MajorPeak();

  // gets the right frequency
  FFT = arduinoFFT(right_ir_reading, imaginary, NUM_SAMPLES, (double) 1/(sample_time*pow(10, -6)));
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  double right_frequency = FFT.MajorPeak();

  
  
  if ((left_frequency > 950) && (left_frequency < 1100)){
    uint32_t left_amplitude = left_max_val - left_min_val;
    
    if (left_amplitude > signalAmplitudeCutOff) {
      current_error += left_amplitude;
    }
  } 

  if((right_frequency > 950) && (right_frequency < 1100)) {
    uint32_t right_amplitude = right_max_val - right_min_val;
    if (right_amplitude > signalAmplitudeCutOff) {
      current_error -= right_amplitude;
    }
  }

  uint32_t total_time = millis() - time_marker;


  double derivative_error = (current_error - last_error_IR);

  total_error_IR += total_time * current_error;


  last_error_IR = current_error;
  
  
  resetMaximums();
  uint32_t turn_value = middle + prop_coef * current_error + derivative_coef * derivative_error + integral_coef * total_error_IR;
  // if (turn_value < max_left_turn) {
  //   return max_left_turn;
  // }
  // if (turn_value > max_right_turn) {
  //   return max_right_turn;
  // }
  return turn_value;
}

uint32_t ir_follow_steering_value() {
  
}

void resetMaximums() {
  left_max_val = 0;
  left_min_val = 1024;
  right_max_val = 0;
  right_min_val = 1024;
}





