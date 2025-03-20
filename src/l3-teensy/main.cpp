#include <Arduino.h>
#include "ballposition.h"

BallPosition ballPosition;
SesorFusion sensorFusion;

void setup() {
    Serial.begin(115200);  

    // UART
    Serial2.begin(115200, SERIAL_8N1, 15, 14);  
}

void loop() {
    Vector ballPos = ballPosition.updatePosition();
    Vector botPos = sensorFusion.updateLocalisation();
    Serial2.print(ballPos.x);  
    Serial2.print(",");
    Serial2.println(ballPos.y);
    Serial2.print(",");
    Serial2.println(botPos.x);
    Serial2.println(",");
    Serial2.println(botPos.y);

    #ifdef DEBUG   
    Serial.print("Ball Position X: ");
    Serial.print(ballPos.x);
    Serial.print(" Y: ");
    Serial.println(ballPos.y);
    Serial.print("Bot Position X: ");
    Serial.print(botPos.x);
    Serial.print(" Y: ");
    Serial.println(botPos.y);
    #endif


    delay(100);  
}