#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
#define trigger_pin PB11
#define echo_pin PB10
int minDistance = 10; //in cm
long duration;
int distance;


void initSonar(){
    Serial.begin(9600);
    pinMode(trigger_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
}

void getDistance(){
    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin, LOW);
    duration = pulseIn(echo_pin, HIGH);
    distance = duration * 0.034/2;
}

bool isPresent(){
    return (distance >= minDistance);
}