#include <Wire.h>

void imuInit();

bool isOnRocks();

void storePosition();

void getPosition();

bool isUpwardsAcceleration();

int16_t getRoll();

int16_t getPitch();

int16_t getYaw();

int16_t convertToDomain(int16_t angle);