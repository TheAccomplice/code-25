#include "imu.h"

// Instantiate the IMU object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setupIMU() {
    if (!bno.begin()) {
        Serial.println("Failed to initialize BNO055 IMU!");
        while (1);
    }
    delay(1000);
}

double readIMUHeading() {
    sensors_event_t event;
    bno.getEvent(&event);
    return event.orientation.x;
}