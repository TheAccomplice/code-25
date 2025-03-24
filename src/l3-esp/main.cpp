#include <Arduino.h>
#include <HardwareSerial.h>
#include <TFLI2C.h>
#include <Wire.h>
#include "PacketSerial.h"
#include "SPI.h"
#include "util.h"
#include "imu.h" // Include IMU header

// initialize serial communication
HardwareSerial MySerial0(0);

#define XSHUT1PIN 0 // pin for shutting down lidar (if needed)
// #define DEBUGTOF // uncomment to enable debugging

// packet serial for lidar communication
PacketSerial L3LIDARSerial;

// initialize lidar sensors
TFLI2C front;
TFLI2C right;
TFLI2C back;
TFLI2C left;
TFLI2C tflI2C[4] = {front, right, back, left};

// array to store distance measurements
int16_t tfDist[4];
int16_t prevDist[4] = {0, 0, 0, 0}; // Store previous distances
int16_t prevSum = 0; // Store previous sum of all distances

// i2c addresses for each lidar sensor
int tfAddress[4] = {0x35, 0x22, 0x33, 0x44}; // front, right, back, left

#define SUM_DROP_THRESHOLD 200 // define how much total drop is considered "sudden"

// Room dimensions
const float ROOM_WIDTH = 158.0;
const float ROOM_LENGTH = 219.0;
float robotX = 0.0, robotY = 0.0;

// structure to store lidar distances
struct lidardata {
    int distance[4];
    float x;
    float y;
};
lidardata esp32lidardata;

// structure for lidar transmission payload
typedef struct lidarTxPayload {
    lidardata esp32lidardata;
} lidarTxPayload;

void setup() {
    Serial.begin(115200); // start serial communication
    MySerial0.begin(115200, SERIAL_8N1, -1, -1); // initialize hardware serial

    L3LIDARSerial.begin(&MySerial0); // initialize packet serial
    Wire.begin(); // initialize i2c communication
    setupIMU(); // Initialize IMU
    
    // save settings for each lidar sensor
    for (int i = 0; i < 4; i++) { 
        tflI2C[i].Save_Settings(tfAddress[i]); 
    }
}

void updatePosition() {
    double heading = readIMUHeading(); // get robot heading in degrees
    double rad = heading * PI / 180.0; // convert to radians

    float frontDist = tfDist[0];
    float rightDist = tfDist[1];
    float backDist = tfDist[2];
    float leftDist = tfDist[3];

    // compute X and Y based on distances from walls
    robotX = (leftDist + (ROOM_WIDTH - rightDist)) / 2.0;
    robotY = (frontDist + (ROOM_LENGTH - backDist)) / 2.0;

    // apply rotation based on IMU heading
    float tempX = robotX * cos(rad) - robotY * sin(rad);
    float tempY = robotX * sin(rad) + robotY * cos(rad);
    robotX = tempX;
    robotY = tempY;

    esp32lidardata.x = robotX;
    esp32lidardata.y = robotY;

    Serial.print("X: "); Serial.print(robotX);
    Serial.print(" Y: "); Serial.println(robotY);
}

void loop() {
    int16_t newSum = 0; // sum of current valid readings

    // read data from each lidar sensor
    for (int i = 0; i < 4; i++) {
        if (tflI2C[i].getData(tfDist[i], tfAddress[i])) { // if data is valid
            newSum += tfDist[i]; // add to sum
        } else {
            tfDist[i] = prevDist[i]; // keep previous reading if read fails
            newSum += prevDist[i];
        } 
    }

    // check if the total sum dropped too much
    if (prevSum != 0 && (prevSum - newSum > SUM_DROP_THRESHOLD)) {
        Serial.println("Ignoring sudden drop in total distance sum!");

        // identify which sensor(s) caused the drop and ignore them
        for (int i = 0; i < 4; i++) {
            if (prevDist[i] - tfDist[i] > (SUM_DROP_THRESHOLD / 4)) {
                Serial.print("Ignoring sensor ");
                Serial.print(i);
                Serial.print(": Prev=");
                Serial.print(prevDist[i]);
                Serial.print(", Current=");
                Serial.println(tfDist[i]);

                tfDist[i] = prevDist[i]; // keep previous value
            }
        }
        newSum = prevSum; // keep previous sum
    }

    // update valid readings
    for (int i = 0; i < 4; i++) {
        esp32lidardata.distance[i] = tfDist[i];
        prevDist[i] = tfDist[i]; // store last valid reading
    }
    prevSum = newSum; // store last valid sum

    // calculate X and Y position
    updatePosition();

    delay(20); // short delay between readings

    // prepare data for transmission
    byte buf[sizeof(lidarTxPayload)];
    memcpy(buf, &esp32lidardata, sizeof(esp32lidardata));
    
    // send lidar data over packet serial
    L3LIDARSerial.send(buf, sizeof(buf));
}