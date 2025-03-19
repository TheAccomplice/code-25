#include <Arduino.h>

#include <array>

#include "PacketSerial.h"
#include "main.h"
//#include "movement.h"
#include "util.h"

void selectMUXChannel(uint8_t channel) {
    digitalWrite(S0, channel & 1);
    digitalWrite(S1, (channel >> 1) & 1);
    digitalWrite(S2, (channel >> 2) & 1);
    digitalWrite(S3, (channel >> 3) & 1);
}
int readMUXChannel(int index) {
    selectMUXChannel(index);
    return analogRead(0);
}

LightArray lightArray;

void getValues() {
  lightArray.RAWLDRVALUES[0] = readMUXChannel(0);
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

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud rate
  while (!Serial);     // Wait for the serial connection to be established
  Serial.println("Starting LDR value read...");
  pinMode(9, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop() {
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);

  Serial.println("Hi");

  delay(1000);  // Wait for 1 second before reading again
}