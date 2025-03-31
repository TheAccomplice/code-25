#include <Arduino.h>
#include "util.h"
#include "main.h"

Point goalPos = {61.5, 0};
Point receivedBallPos;  
Point receivedBotPos;   

SensorValues receivedSensorValues;
Movement movement; // need to define this in .cpp file

#define BALL_POSSESSION_THRESHOLD 10.0  // How close to ball to possess
#define GOAL_APPROACH_THRESHOLD 20.0    // How far from goal before shoot
#define ESCAPE_TURN_ANGLE 90  

void setup() {
    Serial.begin(115200); 
    Serial2.begin(115200);
    Serial3.begin(115200);
}

void loop() {
    if (Serial2.available()) {
        // Read the ball position
        String ballData = Serial2.readStringUntil(',');
        receivedBallPos.x = ballData.toFloat();

        ballData = Serial2.readStringUntil(',');
        receivedBallPos.y = ballData.toFloat();

        // Read the bot position (x and y)
        String botData = Serial2.readStringUntil(',');
        receivedBotPos.x = botData.toFloat();

        botData = Serial2.readStringUntil('\n');
        receivedBotPos.y = botData.toFloat();

        #ifdef DEBUG
        Serial.print("Received Ball Position X: ");
        Serial.print(receivedBallPos.x);
        Serial.print(" Y: ");
        Serial.println(receivedBallPos.y);

        Serial.print("Received Bot Position X: ");
        Serial.print(receivedBotPos.x);
        Serial.print(" Y: ");
        Serial.println(receivedBotPos.y);
        #endif
    }

    if (Serial3.available() >= sizeof(receivedSensorValues)) {
        Serial3.readBytes((char*)&receivedSensorValues, sizeof(receivedSensorValues));
    }

    if (receivedSensorValues.onLine >= 1) {
        // Line detected
        movement.setconstantVelocity(Velocity::constant{0});

        // Escape
        double escapeBearing = receivedSensorValues.angleBisector + ESCAPE_TURN_ANGLE;
        escapeBearing = clipAngleto180degrees(escapeBearing);

        movement.setbearing(Bearing::absolute{escapeBearing});
        movement.setconstantVelocity(Velocity::constant{400});  // Move away
    } 
    else{
      // Distance from robot to ball
      double distanceToBall = sqrt(pow(receivedBallPos.x - receivedBotPos.x, 2) + pow(receivedBallPos.y - receivedBotPos.y, 2));

      // Check if the robot has possession of the ball
      if (distanceToBall < BALL_POSSESSION_THRESHOLD) {
          // Robot has possession of the ball

          // Turn to goal and lock on to it
          double goalBearing = atan2(goalPos.y - receivedBotPos.y, goalPos.x - receivedBotPos.x) * 180 / PI;
          movement.setbearing(Bearing::absolute{goalBearing});  // Face towards the goal

          double distanceToGoal = (goalPos - receivedBotPos).distance;

          if (distanceToGoal > GOAL_APPROACH_THRESHOLD) {
              // Drive to goal
              movement.setmovetoPointDirection(Direction::movetoPoint{goalPos, receivedBotPos});
              movement.setconstantVelocity(Velocity::constant{600});  
          } else {
              // Near to goal, stop promptly to "shoot"
              movement.setconstantVelocity(Velocity::constant{0});  
          }
      } else {
          //No possession of ball, face ball and drive towards it
          double ballBearing = atan2(receivedBallPos.y - receivedBotPos.y, receivedBallPos.x - receivedBotPos.x) * 180 / PI;
          movement.setbearing(Bearing::absolute{ballBearing});

          movement.setmovetoPointDirection(Direction::movetoPoint{receivedBallPos, receivedBotPos});
          movement.setconstantVelocity(Velocity::constant{400});
      }
    }

    movement.drive(receivedBotPos);


    delay(100);
}

