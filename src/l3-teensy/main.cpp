#include <Arduino.h>
#include "ballposition.h"
#include "sensorfusion.h"

BallPosition ballPosition;
SensorFusion sensorFusion;

void safePrintFloat(Stream &serial, float value, const char *label = nullptr) {
    if (label) serial.print(label);
    serial.print(value, 6);
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 15, 14);
    if (!Serial2) {
        Serial.println("Error: Could not initialize Serial2!");
        while (1);
    }
    Serial.println("Setup complete.");
}

void loop() {
    double imuHeading = readIMUHeading();
    if (isnan(imuHeading)) Serial.println("Error: IMU reading is NaN.");

    Vector ballPos = ballPosition.updatePosition();
    Vector botPos = sensorFusion.updateLocalisation();

    safePrintFloat(Serial2, ballPos.x); Serial2.print(",");
    safePrintFloat(Serial2, ballPos.y); Serial2.print(",");
    safePrintFloat(Serial2, botPos.x); Serial2.print(",");
    safePrintFloat(Serial2, botPos.y); Serial2.println();

#ifdef DEBUG
    safePrintFloat(Serial, ballPos.x, "Ball Position X: ");
    Serial.print(" Y: "); safePrintFloat(Serial, ballPos.y); Serial.println();

    safePrintFloat(Serial, botPos.x, "Bot Position X: ");
    Serial.print(" Y: "); safePrintFloat(Serial, botPos.y); Serial.println();
#endif

    delay(100);
}