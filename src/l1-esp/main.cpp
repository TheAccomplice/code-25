#include <Arduino.h>
#include "HardwareSerial.h"
#include <array>

#include "PacketSerial.h"
#include "main.h"
//#include "movement.h"
#include "util.h"


LightArray lightArray; //need to define them in .cpp folder
SensorValues sensorValues;

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

  #define DEBUG
  #ifdef DEBUG
  for (int i = 7; i < 9; i++) {
    lightArray.RAWLDRVALUES[i] = readMUXChannel(i);
    if (lightArray.RAWLDRVALUES[i] <= 0) {
      //return;
    }
    Serial.print(i);
    Serial.print(": ");
    Serial.print(lightArray.RAWLDRVALUES[i]);
    Serial.print(" | ");
  }
  Serial.println();

  delay(100); 
  #endif
}