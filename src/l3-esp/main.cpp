#include <Arduino.h>
#include <HardwareSerial.h>
#include <TFLI2C.h>
#include <Wire.h>
#include "PacketSerial.h"
#include "SPI.h"
#include "util.h"

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

// i2c addresses for each lidar sensor
int tfAddress[4] = {0x35, 0x22, 0x33, 0x44}; // front, right, back, left

// structure to store lidar distances
struct lidardata {
    int distance[4];
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
    
    // save settings for each lidar sensor
    for (int i = 0; i < 4; i++) { 
        tflI2C[i].Save_Settings(tfAddress[i]); 
    }

    // optional settings for modifying i2c addresses (commented out)
    // tflI2C[3].Set_I2C_Addr(0x44,0x22);
    // tflI2C[3].Soft_Reset(0x44);
    // tflI2C[3].Soft_Reset(0x44);
}

void loop() {
    // read data from each lidar sensor
    for (int i = 0; i < 4; i++) {
        if (tflI2C[i].getData(tfDist[i], tfAddress[i])) { // if data is valid
#ifdef DEBUGTOF
            Serial.print("Dist: ");
            Serial.print(tfDist[i]);
            if (i == 3) Serial.println();
            else Serial.print(" ");
#endif
            esp32lidardata.distance[i] = tfDist[i]; // store distance
        } else {
            esp32lidardata.distance[i] = 0; // set to zero if read fails
            tflI2C[i].printStatus(); // print lidar status
        }
    }
    delay(20); // short delay between readings

    // prepare data for transmission
    byte buf[sizeof(lidarTxPayload)];
    memcpy(buf, &esp32lidardata, sizeof(esp32lidardata));
    
    // send lidar data over packet serial
    L3LIDARSerial.send(buf, sizeof(buf));
}