/**
 * @file ir.cpp
 * @brief contains methods for IR sensing and convolution
 *
 *  @copyright Copyright ENPH253 (c) 2023
 */

#include <Wire.h>
#include <Adafruit_SSD1306.h>


#define IR_READ PA0    //digital read pin

#define ADC_READ_PERIOD 1000 // length of time between reads in us.
#define F_READ_PERIOD 1000 // length of time between convultion calculations in us. 
#define NUM_SAMPLES 100 // number of samples which are being collected. 

int getCorrelationFactor(double waveOne[], uint16_t waveTwo[], uint32_t waveTwoStartIndex, int arraySize);

uint16_t ir_reading[NUM_SAMPLES];
// A one kilohertz sine wave. 
double onek[NUM_SAMPLES];
double convolutionMax = 0;
char buffer[100];
uint32_t adc_last_read = 0;
uint32_t f_last_read = 0;
uint32_t max_one = 0;
int counter = 0;
uint32_t cursor = 0;


int ir_init(){
    // fills the sin array with the values of a 1 khZ. 
  for (int j = 0; j < NUM_SAMPLES; j++) {
    onek[j] = sin((double)TWO_PI * 1000 * pow(10,-6) * j);
    ir_reading[j] = 0;
  }
  //attachInterrupt(digitalPinToInterrupt(IO_READ_PIN), interrupt_handler, RISING);
  pinMode(IR_READ, INPUT);
}


void loop() {


  
 
  // if the read period has passed since the last check -> enter body
  if (micros() - adc_last_read >= ADC_READ_PERIOD) {
    
    // if the cursor is greater than the number of samples then we must reset it to 0. 
    if (cursor >= NUM_SAMPLES) {
      cursor = 0;

      // resets the convultion max to 0 becuase we are starting a new cycle. 
      convolutionMax = 0;
    }
    
    adc_last_read = micros();
    ir_reading[cursor] = analogRead(IR_READ);
    
    cursor++;
    
  
  
  }

  //note that this may not work based on the speed to 
  // run the bottom code getting in the way of the top code. 

  // checks if the time has passed for a full read
  if (micros() - f_last_read >= F_READ_PERIOD) {
    uint32_t startTime = micros();
    // assigns the last read to the current time. 
    f_last_read = millis();

    int correlation = getCorrelationFactor(onek, ir_reading, cursor, NUM_SAMPLES);

    
    

    if (correlation > convolutionMax) {
      convolutionMax = correlation;
    }
  }

}

/// @brief Finds the similarity between the two waves. 
/// @param waveOne The stable waves you would like to compare. The two waves must be same size 
/// @param waveTwo The rolling wave you would like to compare. 
/// @param waveTwoStartIndex The index you would like to start on in the rolling wave. 
/// @return A value contianing the a convolution of these two waves
int getCorrelationFactor(double waveOne[], uint16_t waveTwo[], u_int32_t waveTwoStartIndex, int arraySize) {
  int correlation = 0;
    // this is the minimum value for correlation. 
  int sinWaveIterator = 0;
  int readingIterator = waveTwoStartIndex;
    // iterate through the entire sine wave.
  while (sinWaveIterator < arraySize) {
      // resets the reading iterator back to 0. 
    if (readingIterator >= arraySize) {
      readingIterator = 0;
    }
      // multiples the given reading with the sin wave to get the 
    correlation += waveOne[sinWaveIterator] * waveTwo[readingIterator];
    sinWaveIterator++;
    readingIterator++;
  }
  return correlation;
}