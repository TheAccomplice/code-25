#ifndef _IMU_H_
#define _IMU_H_

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Extern IMU instance
extern Adafruit_BNO055 bno;

// Function declarations (NO DEFINITIONS)
void setupIMU();
double readIMUHeading();

#endif  // _IMU_H_