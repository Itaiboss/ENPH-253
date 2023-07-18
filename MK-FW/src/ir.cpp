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

double ir_reading[NUM_SAMPLES] ={0};
double imaginary[NUM_SAMPLES] ={0};
// A one kilohertz sine wave.
uint32_t onek[NUM_SAMPLES*PERIODS];
uint32_t max_val = 0;
uint32_t min_val = 1024;
uint32_t amplitude = 0;
uint32_t correlation[NUM_SAMPLES];
uint32_t max_corr = 0;
uint32_t last_sample = 0;

void ir_init() {
  // fills the sin array with the values of a 1 khZ.
  // double factor = (double)TWO_PI * 1000 * pow(10, -6)* IR_SAMPLE_TIME;
  // for (int i = 0; i < NUM_SAMPLES*PERIODS; i++) {
  //   onek[i] = (sin(factor*i)+1);
  //   //CONSOLE_LOG(LOG_TAG, "ref:%i", onek[i]);
  // }
  pinMode(IR_READ, INPUT);
  CONSOLE_LOG(LOG_TAG, "Initializing IR");
}
void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType);

uint32_t ir_sample() {
  max_corr = 0;
  max_val = 0;
  min_val = 0;
  last_sample = micros();
  uint32_t timer;
  uint32_t sample_time = 0 ;
  for (int i =0; i < NUM_SAMPLES; i++) {
    timer = micros();
    imaginary[i] = 0;
    ir_reading[i] = analogRead(IR_READ);
    if (ir_reading[i] > max_val) {
      max_val = ir_reading[i];
    }
    if (ir_reading[i] < min_val) {
      min_val = ir_reading[i];
    }
    sample_time+= micros()-timer;
  }
  sample_time /= NUM_SAMPLES;
  // //Convolute
  // for (int i = 0; i < NUM_SAMPLES; i++) {
  //     //CONSOLE_LOG(LOG_TAG, "reading:%i", ir_reading[i]);
  //     for (int j = 0; j < NUM_SAMPLES; j++) {
  //         correlation[i]+= ir_reading[j] * onek[i+j];
  //     }
  //     if (correlation[i] > max_corr) {
  //           max_corr = correlation[i];
  //     }
  // }
  arduinoFFT FFT = arduinoFFT(ir_reading,imaginary, NUM_SAMPLES, (double) 1/(sample_time*pow(10, -6)));
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  double frequency = FFT.MajorPeak();
  amplitude = (max_val-min_val)/2;
  CONSOLE_LOG(LOG_TAG, "Time us: %i Amplitude: %i, Frequency: %d", micros()-last_sample, amplitude, (int)frequency);
  if ((frequency > 950) && (frequency < 1100)){
    return amplitude;
  } else {
    return 0;
  }
}