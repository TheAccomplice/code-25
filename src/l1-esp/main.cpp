#include <Arduino.h>

#include <array>

#include "PacketSerial.h"
#include "main.h"
//#include "movement.h"
#include "util.h"

extern LightArray lightArray;

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud rate
  while (!Serial);     // Wait for the serial connection to be established
  Serial.println("Starting LDR value read...");
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop() {
  getValues();
  for (int i = 0; i < LDRPINCOUNT; i++) {

    Serial.print("LDR ");
    Serial.print(i);
    Serial.print(" Value: ");
    Serial.println(lightArray.RAWLDRVALUES[i]);
  }

  Serial.println("Hi");

  delay(5000);  // Wait for 1 second before reading again
}