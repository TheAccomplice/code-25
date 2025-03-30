#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

void setupIMU();
double readIMUHeading();

#endif