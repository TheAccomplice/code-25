#include <Arduino.h>

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud
  while (!Serial);  // Wait for the serial connection to establish
  Serial.println("Hello, World!");  // Print "Hello, World!" to the serial monitor
}

void loop() {
  Serial.println("Hello, World!");
  delay(1000);
}
