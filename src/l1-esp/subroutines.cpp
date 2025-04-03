#include <Arduino.h>

#include <array>

#include "PacketSerial.h"
#include "main.h"
#include "shared.h"
//#include "movement.h"
//#include "util.h"

void selectMUXChannel(uint8_t channel) {
    digitalWrite(S0, channel & 1);
    digitalWrite(S1, (channel >> 1) & 1);
    digitalWrite(S2, (channel >> 2) & 1);
    digitalWrite(S3, (channel >> 3) & 1);
}
int readMUXChannel(int index) {
    selectMUXChannel(index);
    delay(10);
    return analogRead(M1);
}



void getValues() {
  lightArray.RAWLDRVALUES[0] = readMUXChannel(0); //r1
  lightArray.RAWLDRVALUES[1] = readMUXChannel(1);
  lightArray.RAWLDRVALUES[2] = readMUXChannel(2);
  lightArray.RAWLDRVALUES[3] = readMUXChannel(3);
  lightArray.RAWLDRVALUES[4] = readMUXChannel(4);
  lightArray.RAWLDRVALUES[5] = readMUXChannel(5);
  lightArray.RAWLDRVALUES[6] = readMUXChannel(6);
  lightArray.RAWLDRVALUES[7] = readMUXChannel(7);
  lightArray.RAWLDRVALUES[8] = readMUXChannel(8);
  lightArray.RAWLDRVALUES[9] = readMUXChannel(9);
  lightArray.RAWLDRVALUES[10] = readMUXChannel(10);
  lightArray.RAWLDRVALUES[11] = readMUXChannel(11);
  lightArray.RAWLDRVALUES[12] = readMUXChannel(12);
  lightArray.RAWLDRVALUES[13] = readMUXChannel(13);
  lightArray.RAWLDRVALUES[14] = readMUXChannel(14);
}

movingAvg greenAvg[15] = {
    movingAvg(30), movingAvg(30), movingAvg(30), movingAvg(30),
    movingAvg(30), movingAvg(30), movingAvg(30), movingAvg(30),
    movingAvg(30), movingAvg(30), movingAvg(30), movingAvg(30),
    movingAvg(30), movingAvg(30), movingAvg(30)
};
movingAvg whiteAvg[15] = {
    movingAvg(30), movingAvg(30), movingAvg(30), movingAvg(30),
    movingAvg(30), movingAvg(30), movingAvg(30), movingAvg(30),
    movingAvg(30), movingAvg(30), movingAvg(30), movingAvg(30),
    movingAvg(30), movingAvg(30), movingAvg(30)
};

void findLine() {
    getValues(); // Get LDR values

    int maxLDR1 = -1, maxLDR2 = -1; // Two brightest LDRs
    int maxIndex1 = -1, maxIndex2 = -1;
    double maxAngleDiff = 0;

    // Identify the two highest values
    for (int i = 0; i < LDRPINCOUNT; i++) {
        if (lightArray.RAWLDRVALUES[i] > lightArray.LDRThresholds[i]) {
            if (lightArray.RAWLDRVALUES[i] > maxLDR1) {
                maxLDR2 = maxLDR1;
                maxIndex2 = maxIndex1;
                maxLDR1 = lightArray.RAWLDRVALUES[i];
                maxIndex1 = i;
            } else if (lightArray.RAWLDRVALUES[i] > maxLDR2) {
                maxLDR2 = lightArray.RAWLDRVALUES[i];
                maxIndex2 = i;
            }
        }
    }

    // No line detected
    if (maxIndex1 == -1) {
        sensorValues1.onLine = 0; 
        return;
    }

    sensorValues1.onLine = 1; // Line detected

    // If two sensors detect a line, find the angle bisector
    if (maxIndex2 != -1) {
        double angle1 = lightArray.LDRBearings[maxIndex1];
        double angle2 = lightArray.LDRBearings[maxIndex2];

        double angleDiff = abs(angle1 - angle2);
        if (angleDiff > 180) angleDiff = 360 - angleDiff;

        sensorValues1.angleBisector = clipAngleto360degrees((angle1 + angle2) / 2);
        sensorValues1.depthinLine = 1.0 - cosf(angleDiff / 2.0 * PI / 180.0);
    } else {
        // Only one sensor detected the line, assume direct alignment
        sensorValues1.angleBisector = lightArray.LDRBearings[maxIndex1];
        sensorValues1.depthinLine = 0;
    }
}
