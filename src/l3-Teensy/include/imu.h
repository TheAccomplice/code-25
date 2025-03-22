#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <ArduinoEigenDense.h>
// Global variables for IMU


void setupIMU();
double readIMUHeading();

#endif