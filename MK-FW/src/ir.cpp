/**
 * @file ir.cpp
 * @brief contains methods for IR sensing and convolution
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */
#include <ir.h>
#include <logs.h>

static const char* LOG_TAG = "IR";

uint32_t ir_reading[NUM_SAMPLES];
uint32_t ir_reading_cpy[NUM_SAMPLES];
// A one kilohertz sine wave.
double onek[NUM_SAMPLES];
double convolutionMax = 0;
char buffer[100];
uint32_t adc_last_read = 0;
uint32_t f_last_read = 0;
uint32_t max_one = 0;
int counter = 0;
uint32_t cursor = 0;
uint32_t max_val = 0;
uint32_t min_val = 1024;
uint32_t peak;
#define PERIODS 2


void ir_init() {
  // fills the sin array with the values of a 1 khZ.
  double factor = (double)TWO_PI * 1000 * pow(10,-6)* ADC_READ_PERIOD;
  for (int j = 0; j < NUM_SAMPLES*PERIODS; j++) {
    onek[j] = sin(factor*j);
    ir_reading[j] = 0;
  }
  pinMode(IR_READ, INPUT);
  //pinMode(IR_RESET, OUTPUT);
  CONSOLE_LOG(LOG_TAG, "Initializing IR");
}

void ir_sample() {
  //if the read period has passed since the last check -> enter body
  if (micros() - adc_last_read >= ADC_READ_PERIOD) {
    // if the cursor is greater than the number of samples then we must reset it to 0.
    if (cursor >= NUM_SAMPLES) {
      cursor = 0;
      // resets the convultion max to 0 becuase we are starting a new cycle.
      convolutionMax = 0;
    //   max_val = 0;
    //   min_val = 1024;
    }
    adc_last_read = micros();
    ir_reading[cursor] = analogRead(IR_READ);
    // if (ir_reading[cursor] > max_val) {
    //     max_val = ir_reading[cursor];
    // }
    // if (ir_reading[cursor] < min_val) {
    //     min_val = ir_reading[cursor];
    // }
    // peak = (max_val-min_val)/2;
    cursor++;
  }
  //note that this may not work based on the speed to
  // run the bottom code getting in the way of the top code.
  // checks if the time has passed for a full read
  if (micros() - f_last_read >= F_READ_PERIOD) {
    memcpy(ir_reading_cpy,ir_reading,sizeof(ir_reading));
    // assigns the last read to the current time.
    f_last_read = micros();

    int correlation = getCorrelationFactor(onek, ir_reading_cpy, NUM_SAMPLES*PERIODS, NUM_SAMPLES);
    CONSOLE_LOG(LOG_TAG, "IR_max:%i,IR_min:%i, IR_peak:%i, correlation:%i", max_val, min_val, peak, correlation);
  }
}

/// @brief Finds the similarity between the two waves.
/// @param waveOne The stable waves you would like to compare. The two waves must be same size
/// @param waveTwo The rolling wave you would like to compare.
/// @param waveTwoStartIndex The index you would like to start on in the rolling wave.
/// @return A value contianing the a convolution of these two waves
int getCorrelationFactor(double waveOne[], uint32_t waveTwo[],int waveOneSize ,int waveTwoSize) {
    uint32_t max_corr = 0; 
    uint32_t correlation[waveOneSize];
    for (int i = 0; i < waveOneSize; i++) {
        for (int j = 0; j < waveTwoSize; j++) {
            correlation[i]+= waveTwo[i+1] + waveOne[j];
        }
    }
    for (int i = 0; i < waveOneSize; i++) {
        if (max_corr > correlation[i]) {
            max_corr = correlation[i];
        }
    }
    return max_corr;
}