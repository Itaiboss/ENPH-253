/**
 * @file ir.cpp
 * @brief contains methods for IR sensing and convolution
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */
#include <ir.h>
#include <logs.h>
#include <control.h>
#include <arduinoFFT.h>

static const char* LOG_TAG = "IR";

double left_ir_reading[NUM_SAMPLES] = {0};
double right_ir_reading[NUM_SAMPLES] = {0};
double extreme_left_ir_reading[NUM_SAMPLES] = {0};
double extreme_right_ir_reading[NUM_SAMPLES] = {0};


uint32_t signalAmplitudeCutOff = 50;
uint32_t bottom_frequency_cutoff = 950;
uint32_t expected_frequency = 1000;
uint32_t top_frequency_cutoff = 1150;
bool does_need_frequency_check = true;
uint32_t trials_since_last_check = 0;

double bias = 1.05;

// A one kilohertz sine wave.
uint32_t left_max_val = 0;
uint32_t left_min_val = 1024;
uint32_t right_max_val = 0;
uint32_t right_min_val = 1024;
uint32_t extreme_left_max_val = 0;
uint32_t extreme_left_min_val = 1024;
uint32_t extreme_right_max_val = 0;
uint32_t extreme_right_min_val = 1024;
uint32_t left_amplitude = 0;
uint32_t right_amplitude = 0;
uint32_t left_extreme_ampltude = 0;
uint32_t right_extreme_amplitude = 0;

// the following variables are used to track the 
// different prop, derivative, and integral error terms. 
int32_t current_error = 0;
int32_t last_error_IR = 0;
int32_t total_error_IR = 0;

// the following are coefficents to control the PID logic 
double prop_coef = 0.001;
double derivative_coef = 0;
double integral_coef = 0;


uint32_t centeredWeight = 1;
uint32_t outsideWeight = 5;

uint32_t frequency_check_frequency = 0;

void ir_init() {
  pinMode(IR_R, INPUT);
  pinMode(IR_L, INPUT);
  pinMode(IR_E_L, INPUT);
  pinMode(IR_E_R, INPUT);
  CONSOLE_LOG(LOG_TAG, "Initializing IR");
  does_need_frequency_check = true;
  trials_since_last_check = 0;
  // gives the default values for 
  
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

    if(l_e > extreme_left_max_val) {
      extreme_left_max_val = l_e;
    }

    if(l_e < extreme_left_min_val) {
      extreme_left_min_val = l_e;
    }

    if(r_e > extreme_right_max_val) {
      extreme_right_max_val = r_e;
    }
    if(r_e < extreme_right_min_val) {
      extreme_right_min_val = r_e;
    }

}

uint32_t getFrequency(double ir_reading[], uint32_t num_of_samples, uint32_t time) {
  double imaginary[NUM_SAMPLES];
  for (int index = 0; index < num_of_samples; index++) {
    imaginary[index] = 0;
  }
  arduinoFFT FFT = arduinoFFT(ir_reading, imaginary, NUM_SAMPLES, (double) 1/(time*pow(10, -6)));
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  return FFT.MajorPeak();
}

uint32_t getFrequency(sensors whichSensor, uint32_t sample_time) {

  switch (whichSensor) {
    case left: 
      return getFrequency(left_ir_reading, NUM_SAMPLES, sample_time);
    case right: 
      return getFrequency(right_ir_reading, NUM_SAMPLES, sample_time);
    case extreme_left: 
      return getFrequency(extreme_left_ir_reading, NUM_SAMPLES, sample_time);
    case extreme_right: 
      return getFrequency(extreme_right_ir_reading, NUM_SAMPLES, sample_time);
  }
  
}
bool isFrequencyValid(uint32_t frequency) {
  return frequency < TOP_FREQUENCY_CUTOFF && frequency > BOTTOM_FREQUENCY_CUTOFF;
}


void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType);


bool IR_present() {

  uint32_t timer;
  uint32_t time_marker = millis();
  uint32_t sample_time = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    timer = micros();
    left_ir_reading[i] = analogRead(IR_L);
    right_ir_reading[i] = analogRead(IR_R);
    extreme_left_ir_reading[i] = analogRead(IR_E_L);
    extreme_right_ir_reading[i] = analogRead(IR_E_R);

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

  left_amplitude = 0;
  right_amplitude = 0;
  left_extreme_ampltude = 0;
  right_extreme_amplitude = 0;

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
  CONSOLE_LOG(LOG_TAG, "FREQ l: %i, r: %i, le: %i, re: %i", (int) left_frequency, (int) right_frequency, (int) left_extreme_frequency, (int) right_extreme_frequency);
  CONSOLE_LOG(LOG_TAG, "AMP l: %i, r: %i, le: %i, re: %i", (int) left_amplitude, (int) right_amplitude, (int) left_extreme_ampltude, (int) right_extreme_amplitude);


    if (left_amplitude > MIN_DETECTION_AMPLITUDE) {
      resetMaximums();
      return true;
    }

    if (right_amplitude > MIN_DETECTION_AMPLITUDE) {
      resetMaximums();
      return true;
    }

    if (left_extreme_ampltude > MIN_DETECTION_AMPLITUDE) {
      resetMaximums();
      return true;
    }

    if (right_extreme_amplitude > MIN_DETECTION_AMPLITUDE) {
      resetMaximums();
      return true;
    }

  resetMaximums();

  return false;

}

void ir_PID() {

  
  uint32_t timer;
  uint32_t time_marker = millis();
  uint32_t sample_time = 0;
  current_error = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    timer = micros();
    left_ir_reading[i] = analogRead(IR_L);
    right_ir_reading[i] = analogRead(IR_R);
    extreme_left_ir_reading[i] = analogRead(IR_E_L);
    extreme_right_ir_reading[i] = analogRead(IR_E_R);

    // CONSOLE_LOG(LOG_TAG, "l: %i, r: %i, le: %i, re: %i", (int) left_ir_reading[i], (int) right_ir_reading[i], (int) extreme_left_ir_reading[i], (int) extreme_right_ir_reading[i]);

    
    updateMinAndMax(left_ir_reading[i],
     right_ir_reading[i],
     extreme_left_ir_reading[i],
     extreme_right_ir_reading[i]);

    
    
    sample_time += micros() - timer;
  }

  // CONSOLE_LOG(LOG_TAG, "le: %i %i, re: %i - %i", (int) extreme_left_max_val, (int) extreme_left_min_val, (int) extreme_right_max_val, (int) extreme_left_min_val);

  sample_time = sample_time / NUM_SAMPLES;
  double left_frequency = expected_frequency;
  double right_frequency = expected_frequency;
  double left_extreme_frequency = expected_frequency;
  double right_extreme_frequency = expected_frequency;

  left_frequency = getFrequency(left, sample_time);
    right_frequency = getFrequency(right, sample_time);
    left_extreme_frequency = getFrequency(extreme_left, sample_time);
    right_extreme_frequency = getFrequency(extreme_right, sample_time);

  // if (does_need_frequency_check) {
  //   left_frequency = getFrequency(left, sample_time);
  //   right_frequency = getFrequency(right, sample_time);
  //   left_extreme_frequency = getFrequency(extreme_left, sample_time);
  //   right_extreme_frequency = getFrequency(extreme_right, sample_time);
  //   does_need_frequency_check = false;
  //   trials_since_last_check = 0;
  // } else {
  //   if (trials_since_last_check == frequency_check_frequency) {
  //     does_need_frequency_check = true;
  //   } else {
  //     trials_since_last_check++;
  //   }
  // }

  left_amplitude = 0;
  right_amplitude = 0;
  left_extreme_ampltude = 0;
  right_extreme_amplitude = 0;


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

  if (left_amplitude < signalAmplitudeCutOff) {
    left_amplitude = 0;
  }
  if (right_amplitude < signalAmplitudeCutOff) {
    right_amplitude = 0;
  }
  if (left_extreme_ampltude < signalAmplitudeCutOff) {
    left_extreme_ampltude = 0;
  }
  if (right_extreme_amplitude < signalAmplitudeCutOff) {
    right_extreme_amplitude = 0;
  }

  //CONSOLE_LOG(LOG_TAG, "r: %i, e: %i, l: %i, e: %i", (int) right_amplitude, (int) right_extreme_amplitude, (int) left_amplitude, (int) left_extreme_ampltude);

  

  // CONSOLE_LOG(LOG_TAG, "frequency: %i, amp: %i", (int) right_extreme_frequency, (int) right_extreme_amplitude);
  // CONSOLE_LOG(LOG_TAG, "right max: %i right min: %i",  (int) extreme_right_max_val, (int) extreme_right_min_val);

  current_error = get_error(right_amplitude, left_amplitude, right_extreme_amplitude, left_extreme_ampltude);

  uint32_t total_time = millis() - time_marker;
  uint32_t derivative_error = (current_error - last_error_IR);
  total_error_IR += total_time * current_error;
  last_error_IR = current_error;

  resetMaximums();

  
  uint32_t turn_value = MID_POINT + prop_coef * current_error + derivative_coef * derivative_error + integral_coef * total_error_IR;
  //CONSOLE_LOG(LOG_TAG, "error is: %i, turn Value is: %i", (int) current_error, (int) turn_value);
  set_raw_steering(turn_value);
 
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

int32_t get_error(uint32_t right, uint32_t left, uint32_t right_extreme, uint32_t left_extreme) {
  // CONSOLE_LOG(LOG_TAG, "right: %i, left: %i", (int) right, (int) left);
  double total = right + left + right_extreme + left_extreme;

  int32_t normalized_right = normalize_magnitude(total, right) * bias;
  int32_t normalized_left = normalize_magnitude(total, left) / bias ;
  int32_t normalized_e_right = normalize_magnitude(total, right_extreme) * bias;
  int32_t normalized_e_left = normalize_magnitude(total, left_extreme) / bias;

  return centeredWeight * (normalized_left - normalized_right) + outsideWeight * (normalized_e_left - normalized_e_right);
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
  return ampltitude / (total + 100) * 10000;
}


void resetErrors() {
  current_error = 0;
  last_error_IR = 0;
  total_error_IR = 0;
}

/**
 * @brief  Resets all the maximum values. 
 * @note   
 * @retval None
 */
void resetMaximums() {
  left_max_val = 0;
  left_min_val = 1024;
  right_max_val = 0;
  right_min_val = 1024;
  extreme_left_max_val = 0;
  extreme_left_min_val = 1024;
  extreme_right_max_val = 0;
  extreme_right_min_val = 1024;
}






