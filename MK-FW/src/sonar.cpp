#include <Wire.h>
 
#define trigger_pin PB11
#define echo_pin PB10
int minDistance = 10; //in cm
long duration;
uint32_t distance;


void initSonar() {
    Serial.begin(9600);
    pinMode(trigger_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
}

uint32_t getDistance() {
    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin, LOW);
    duration = pulseIn(echo_pin, HIGH);
    distance = duration * 0.034/2;
    return distance;
}

bool isPresent() {
    return (distance >= minDistance);
}