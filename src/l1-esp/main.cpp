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
  Serial.begin(115200);  // Initialize serial communication at 9600 baud rate
  Serial0.begin(115200, SERIAL_8N1, 7, 6); // esp32 only has UART0
}

void loop() {
  findLine();

  Serial0.write((byte*)&sensorValues, sizeof(sensorValues));

  delay(100);



  #ifdef DEBUG
  getValues();
  for (int i = 0; i < LDRPINCOUNT; i++) {

    Serial.print("LDR ");
    Serial.print(i);
    Serial.print(" Value: ");
    Serial.println(lightArray.RAWLDRVALUES[i]);
  }

  Serial.println("Hi");

  delay(5000);  // Wait for 1 second before reading again
  #endif
}