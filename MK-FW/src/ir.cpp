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
double extreme_left_ir_reading[NUM_SAMPLES] = {0};
double extreme_right_ir_reading[NUM_SAMPLES] = {0};
double l_imaginary[NUM_SAMPLES];
double l_e_imaginary[NUM_SAMPLES];
double r_e_imaginary[NUM_SAMPLES];
double r_imaginary[NUM_SAMPLES];
uint32_t valueHistory[ROLLING_AVERAGE_NUMBER] = {0};
uint32_t signalAmplitudeCutOff = 20;
uint32_t bottom_frequency_cutoff = 950;
uint32_t top_frequency_cutoff = 1150;

// A one kilohertz sine wave.
uint32_t left_max_val = 0;
uint32_t left_min_val = 1024;
uint32_t right_max_val = 0;
uint32_t right_min_val = 1024;
uint32_t extreme_left_max_val = 0;
uint32_t extreme_left_min_val = 1024;
uint32_t extreme_right_max_val = 0;
uint32_t extreme_right_min_val = 1024;



// the following variables are used to track the 
// different prop, derivative, and integral error terms. 
double current_error = 0;
double last_error_IR = 0;
double total_error_IR = 0;

// the following are coefficents to control the PID logic 
double prop_coef = 100;
double derivative_coef = 0;
double integral_coef = 0;

uint32_t max_left_turn = 190;
uint32_t max_right_turn = 330;
uint32_t middle = 267;

uint32_t centeredWeight = 1;
uint32_t outsideWeight = 4;


void ir_init() {
  pinMode(IR_READ_LEFT, INPUT);
  pinMode(IR_READ_RIGHT, INPUT);
  pinMode(IR_READ_EXTREME_LEFT, INPUT);
  pinMode(IR_READ_EXTREME_RIGHT, INPUT);
  CONSOLE_LOG(LOG_TAG, "Initializing IR");
  // gives the default values for 
  for (int i = 0; i < ROLLING_AVERAGE_NUMBER; i++) {
    valueHistory[i] = middle;
  }
}

void reset_imaginary() {
  for(int i = 0; i < NUM_SAMPLES; i++) {
    l_imaginary[i] = 0;
    l_e_imaginary[i] = 0;
    r_e_imaginary[i] = 0;
    r_imaginary[i] = 0;
  }
}

void updateMinAndMax(int l, int r, int l_e, int r_e) {
  if(l > left_max_val) {
      left_max_val = l;
    }

    if(l < left_min_val) {
      left_min_val = l;
    }

    if(r > right_max_val) {
      right_max_val = r;
    }
    if(r < right_min_val) {
      right_min_val = r;
    }

    if(l_e > left_max_val) {
      left_max_val = l_e;
    }

    if(l_e < left_min_val) {
      left_min_val = l_e;
    }

    if(r_e > right_max_val) {
      right_max_val = r_e;
    }
    if(r_e < right_min_val) {
      right_min_val = r_e;
    }

}

uint32_t getFrequency(sensors whichSensor, uint32_t sample_time) {

  reset_imaginary();
  
  if (whichSensor == left) {
      arduinoFFT FFT_left = arduinoFFT(left_ir_reading, l_imaginary, NUM_SAMPLES, (double) 1/(sample_time*pow(10, -6)));
      FFT_left.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
      FFT_left.Compute(FFT_FORWARD); /* Compute FFT */
      FFT_left.ComplexToMagnitude(); /* Compute magnitudes */
      return FFT_left.MajorPeak();
  }

  if (whichSensor == right) {
    arduinoFFT FFT_right = arduinoFFT(right_ir_reading, r_imaginary, NUM_SAMPLES, (double) 1/(sample_time*pow(10, -6)));      
    FFT_right.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT_right.Compute(FFT_FORWARD); /* Compute FFT */
    FFT_right.ComplexToMagnitude(); /* Compute magnitudes */
    return FFT_right.MajorPeak();
  }

  if (whichSensor == extreme_left) {
    arduinoFFT FFT_right = arduinoFFT(extreme_left_ir_reading, l_e_imaginary, NUM_SAMPLES, (double) 1/(sample_time*pow(10, -6)));      
    FFT_right.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT_right.Compute(FFT_FORWARD); /* Compute FFT */
    FFT_right.ComplexToMagnitude(); /* Compute magnitudes */
    return FFT_right.MajorPeak();
  }  

    arduinoFFT FFT_right = arduinoFFT(extreme_right_ir_reading, r_e_imaginary, NUM_SAMPLES, (double) 1/(sample_time*pow(10, -6)));      
    FFT_right.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT_right.Compute(FFT_FORWARD); /* Compute FFT */
    FFT_right.ComplexToMagnitude(); /* Compute magnitudes */
    return FFT_right.MajorPeak();
}

bool isFrequencyValid(uint32_t frequency) {
  return frequency < top_frequency_cutoff && frequency > bottom_frequency_cutoff;
}


void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType);

uint32_t ir_PID() {
  
  uint32_t timer;
  uint32_t time_marker = millis();
  uint32_t sample_time = 0;
  current_error = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    timer = micros();
    left_ir_reading[i] = analogRead(IR_READ_LEFT);
    right_ir_reading[i] = analogRead(IR_READ_RIGHT);
    extreme_left_ir_reading[i] = analogRead(IR_READ_EXTREME_LEFT);
    extreme_right_ir_reading[i] = analogRead(IR_READ_EXTREME_RIGHT);

    updateMinAndMax(left_ir_reading[i],
     right_ir_reading[i],
     extreme_left_ir_reading[i],
     extreme_right_ir_reading[i]);
    
    sample_time += micros() - timer;
  }

  sample_time = sample_time / NUM_SAMPLES;
  double left_frequency = getFrequency(left, sample_time);
  double right_frequency = getFrequency(right, sample_time);
  double left_extreme_frequency = getFrequency(extreme_left, sample_time);
  double right_extreme_frequency = getFrequency(extreme_right, sample_time);

  uint32_t left_amplitude = 0;
  uint32_t right_amplitude = 0;
  uint32_t left_extreme_ampltude = 0;
  uint32_t right_extreme_amplitude = 0;


  if (isFrequencyValid(left_frequency)) {
    left_amplitude = left_max_val - left_min_val;
  }
  if (isFrequencyValid(right_frequency)) {
    right_amplitude = right_max_val - right_min_val;
  }
  if (isFrequencyValid(left_extreme_frequency)) {
    left_extreme_ampltude = extreme_left_max_val - extreme_left_min_val;
  }
  if (isFrequencyValid(right_extreme_frequency)) {
    right_extreme_amplitude = extreme_right_max_val - extreme_right_min_val;
  }

  // CONSOLE_LOG(LOG_TAG, "right: %i, left: %i", (int) right_amplitude, (int) left_amplitude);
 

  current_error = get_error(right_amplitude, left_amplitude);

  uint32_t total_time = millis() - time_marker;
  double derivative_error = (current_error - last_error_IR);
  total_error_IR += total_time * current_error;
  last_error_IR = current_error;

  resetMaximums();
  uint32_t turn_value = middle + prop_coef * current_error + derivative_coef * derivative_error + integral_coef * total_error_IR;
  if (turn_value < max_left_turn) {
    return max_left_turn;
  }
  if (turn_value > max_right_turn) {
    return max_right_turn;
  }
  return turn_value;
}




/**
 * @brief  Gets the error where positive refers to the left direction and negative to the left direction. 
 * @note   
 * @param  right: The amplitude of the right IR detector.
 * @param  left: The amplitude of the left IR detector. 
 * @param  right_extreme: The amplitude of the outside right IR detector 
 * @param  left_extreme: The amplitude of the outside left IR detector
 * @retval A double containg the value of the error. 
 */

double get_error(uint32_t right, uint32_t left, uint32_t right_extreme, uint32_t left_extreme) {
  // CONSOLE_LOG(LOG_TAG, "right: %i, left: %i", (int) right, (int) left);
  double total = right + left + right_extreme + left_extreme;

  double normalized_right = normalize_magnitude(total, right);
  double normalized_left = normalize_magnitude(total, left);
  double normalized_e_right = normalize_magnitude(total, right_extreme);
  double normalized_e_left = normalize_magnitude(total, left_extreme);

  return centeredWeight * (normalized_left - normalized_right) + centeredWeight * (normalized_e_left - normalized_e_right);
}

/**
 * @brief  Normalizes the magnitude based on the distance to provide a different maginitude.
 * @note   
 * @param  total: the total magnitude of the left and right waves
 * @param  ampltitude: The magnitude if the wave that we care about. 
 * @retval A unsigned int value that is the normalized value. 
 */
double normalize_magnitude(double total, uint32_t ampltitude) {
  // CONSOLE_LOG(LOG_TAG, "total: %i, ampltiude: %i, divided term: %i", (int) total, (int) ampltitude, (int) (ampltitude / total * 100));
  return ampltitude / (total + 100);
}



void resetMaximums() {
  left_max_val = 0;
  left_min_val = 1024;
  right_max_val = 0;
  right_min_val = 1024;
}





