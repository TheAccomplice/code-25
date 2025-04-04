#include <Arduino.h>
#include <HardwareSerial.h>
#include <TFLI2C.h>
#include <Wire.h>

#include "PacketSerial.h"
#include "util.h"

//HardwareSerial Serial0(0);

#define XSHUT1PIN 0
// #define DEBUGTOF

PacketSerial L3LIDARSerial;
TFLI2C front;
TFLI2C right;
TFLI2C back;
TFLI2C left;
TFLI2C tflI2C[4] = {front, right, back, left};

int16_t tfDist[4];

int tfAddress[4] = {0x12, 0x13, 0x10, 0x11}; // front, right, back, left

struct lidardata {
    int distance[4];
};
lidardata esp32lidardata;

typedef struct lidarTxPayload {
    lidardata esp32lidardata;
} lidarTxPayload;

void setup() {
    Serial.begin(115200);
    Serial0.begin(115200, SERIAL_8N1, D7, D6);

    L3LIDARSerial.begin(&Serial0);
    Wire.begin();
    for (int i = 0; i < 4; i++) { tflI2C[i].Save_Settings(tfAddress[i]); }

    // tflI2C[3].Set_I2C_Addr(0x44,0x22);
    // tflI2C[3].Soft_Reset(0x44);
    // tflI2C[3].Soft_Reset(0x44);
}

void loop() {
    // i2cscanner();
    for (int i = 0; i < 4; i++) {
        if (tflI2C[i].getData(tfDist[i], tfAddress[i])) // If read okay...
        {
#define DEBUGTOF
#ifdef DEBUGTOF
            if (i < 3) {
                Serial.print(" Dist: ");
                Serial.print(tfDist[i]);
            }

            else if (i == 3) {
                Serial.print(" Dist: ");
                Serial.println(tfDist[i]);
            }
#endif
            esp32lidardata.distance[i] = tfDist[i];
        }

        else {
            esp32lidardata.distance[i] = 0;
            tflI2C[i].printStatus();
        }
    }
    delay(20);

    byte buf[sizeof(lidarTxPayload)];
    memcpy(buf, &esp32lidardata, sizeof(esp32lidardata));
    L3LIDARSerial.send(buf, sizeof(buf));
}





























// #include <Arduino.h>
// #include <Wire.h>
// #include <TFLI2C.h>
// #include "PacketSerial.h"
// #include "SPI.h"
// #include "util.h"
// #include "imu.h"

// // Use Serial1 (or change to Serial2/3 based on wiring)
// #define LIDAR_SERIAL Serial1

// #define NUM_SENSORS 4
// #define SUM_DROP_THRESHOLD 200
// #define MAX_VALID_DISTANCE 3000  // Max valid LIDAR distance in mm
// #define MIN_VALID_DISTANCE 0

// #define ROOM_WIDTH 158.0
// #define ROOM_LENGTH 219.

// TFLI2C front, right, back, left;
// TFLI2C* tflI2C[NUM_SENSORS] = {&front, &right, &back, &left};
// const uint8_t tfAddress[NUM_SENSORS] = {0x12, 0x13, 0x10, 0x11};

// int16_t tfDist[NUM_SENSORS];
// int16_t prevDist[NUM_SENSORS] = {0, 0, 0, 0};
// int16_t prevSum = 0;

// PacketSerial L3LIDARSerial;

// float robotX = 0.0, robotY = 0.0;

// struct __attribute__((packed)) lidardata {
//     int16_t distance[NUM_SENSORS];
//     float x;
//     float y;
// };

// lidardata esp32lidardata;

// void setup() {
//     Seria0.begin(115200);
//     LIDAR_SERIAL.begin(115200);

//     L3LIDARSerial.setStream(&LIDAR_SERIAL);
//     Wire.begin(); 

//     setupIMU();

//     // Save I2C addresses (Optional, ensure this works properly)
//     // for (int i = 0; i < NUM_SENSORS; i++) {
//     //     tflI2C[i]->Save_Settings(tfAddress[i]);
//     // }
// }

// void updatePosition() {
//     double heading = readIMUHeading();
//     heading = fmod(heading, 360.0);
//     if (heading < 0) heading += 360.0;
//     double rad = heading * PI / 180.0;

//     float frontDist = tfDist[0];
//     float rightDist = tfDist[1];
//     float backDist = tfDist[2];
//     float leftDist = tfDist[3];

//     // Compute raw X, Y
//     robotX = (leftDist + (ROOM_WIDTH - rightDist)) / 2.0;
//     robotY = (frontDist + (ROOM_LENGTH - backDist)) / 2.0;

//     // Apply IMU rotation correction
//     float tempX = robotX * cos(rad) - robotY * sin(rad);
//     float tempY = robotX * sin(rad) + robotY * cos(rad);
//     robotX = tempX;
//     robotY = tempY;

//     esp32lidardata.x = robotX;
//     esp32lidardata.y = robotY;

//     Serial.print("X: "); Serial.print(robotX);
//     Serial.print(" Y: "); Serial.println(robotY);
// }

// void loop() {
//     int16_t newSum = 0;

//     for (int i = 0; i < NUM_SENSORS; i++) {
//         if (tflI2C[i]->getData(tfDist[i], tfAddress[i])) {
//             // Sanity check distance
//             if (tfDist[i] < MIN_VALID_DISTANCE || tfDist[i] > MAX_VALID_DISTANCE) {
//                 tfDist[i] = prevDist[i];
//             }
//             newSum += tfDist[i];
//         } else {
//             tfDist[i] = prevDist[i];
//             newSum += prevDist[i];
//         }
//     }

//     // Sudden drop filtering
//     if (prevSum != 0 && (prevSum - newSum > SUM_DROP_THRESHOLD)) {
//         Serial.println("Ignoring sudden drop in total distance sum!");
//         for (int i = 0; i < NUM_SENSORS; i++) {
//             if (prevDist[i] - tfDist[i] > (SUM_DROP_THRESHOLD / NUM_SENSORS)) {
//                 Serial.print("Ignoring sensor "); Serial.print(i);
//                 Serial.print(": Prev="); Serial.print(prevDist[i]);
//                 Serial.print(", Current="); Serial.println(tfDist[i]);
//                 tfDist[i] = prevDist[i];
//             }
//         }
//         newSum = prevSum;
//     }

//     // Update distances
//     for (int i = 0; i < NUM_SENSORS; i++) {
//         esp32lidardata.distance[i] = tfDist[i];
//         prevDist[i] = tfDist[i];
//     }
//     prevSum = newSum;

//     updatePosition();

//     // Send data as packed struct
//     L3LIDARSerial.send((const uint8_t*)&esp32lidardata, sizeof(esp32lidardata));

//     delay(20);
// }