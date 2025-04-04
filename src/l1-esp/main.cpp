#include <Arduino.h>
#include "HardwareSerial.h"
#include <array>

#include "PacketSerial.h"
#include "main.h"

//#include "movement.h"
#include "util.h"
#include "shared.h"


LightArray lightArray; //need to define them in .cpp folder
SensorValues1 sensorValues1;

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(M1, INPUT);
  analogReadResolution(12);
  Serial.begin(115200);  // Initialize serial communication at 9600 baud rate
  Serial0.begin(115200, SERIAL_8N1, D7, D6); // esp32 only has UART0
}

void loop() {
  
  /*
  findLine();

  Serial0.write((byte*)&sensorValues, sizeof(sensorValues));

  delay(100);
  */
  for (int i = 0; i < 15; i++) {
    lightArray.RAWLDRVALUES[i] = readMUXChannel(i);
    if (lightArray.RAWLDRVALUES[i] > lightArray.LDRThresholds[i]) {
      whiteAvg[i].push(lightArray.RAWLDRVALUES[i]);
    }
    else{
      greenAvg[i].push(lightArray.RAWLDRVALUES[i]);
    }

    lightArray.LDRThresholds[i] = (whiteAvg[i].get_avg() + greenAvg[i].get_avg()) / 2;

    Serial.print(i);
    Serial.print(": ");
    Serial.print(lightArray.RAWLDRVALUES[i]);
    Serial.print(" | ");
    Serial.print("T: ");
    Serial.print(lightArray.LDRThresholds[i]);
    Serial.print(" | ");
    


  }
Serial.println();
delay(100);
}