#include <Arduino.h>
#include <PacketSerial.h>
#include "util.h"
#include "main.h"
#include "shared.h"

PacketSerial TeensyTeensySerial;
PacketSerial ESPSerial;
Point goalPos = {61.5, 0};
Point receivedBallPos;  
Point receivedBotPos;   
SensorValues1 sensorValues1;


//CONFIG ******************
// Ball Cruve Settings
// https://www.desmos.com/calculator/atvzoxtoxn
#define OFFSET_MULTIPLIER 0.04F // variable a in desmos
#define START_OFFSET      28    // variable d
#define DEGREE_MULTIPLIER 20
//*************** */

// struct ProcessedValues {
//     //relative to robot
//     Vector ball_relativeposition;
//     Vector yellowgoal_relativeposition;
//     Vector bluegoal_relativeposition;
//     //relative to field, not used?
//     Vector ball_actualposition;
//     Vector yellowgoal_actualposition;
//     Vector bluegoal_actualposition;

//     int ballExists = 0;
//     int yellowgoal_exists = 0;
//     int bluegoal_exists = 0;
//     int lidarDistance[4];
//     double lidarConfidence[4];
//     double bearing_relative_to_field;
//     int relativeBearing;
//     Point robot_position;
//     int is_ball_in_catchment = 0;
//     int onLine = 0;
//     float angleBisector;
//     float depthinLine;
//     double depth_in_line;
// };

ProcessedValues processedValues;
Movement movement; // need to define this in .cpp file

#define BALL_POSSESSION_THRESHOLD 10.0  // How close to ball to possess
#define GOAL_APPROACH_THRESHOLD 20.0    // How far from goal before shoot
#define ESCAPE_TURN_ANGLE 90  


// DO SERIAL COMMUNICATION HERE ************



//***************** */


double ballAngleOffset(double distance, double direction) {
    // offset multiplier https://www.desmos.com/calculator/8d2ztl2zf8

    if (direction < 50) {
        double constant = -5;
        double angleoffset =
            constrain(direction * 1, -90, 90) *
            fmin(powf(exp(1), OFFSET_MULTIPLIER * (START_OFFSET - distance)),
                 1);
        return angleoffset;
    } else {
        double angleoffset =
            constrain(direction * 1, -90, 90) *
            fmin(powf(exp(1), OFFSET_MULTIPLIER * (START_OFFSET - distance)),
                 1);
        return angleoffset;
    }
};

double curveAroundBallMultiplier(double angle, double actual_distance,
                                 double start_distance) {
    double radial_distance =
        sqrtf(powf(angle, 2)) / 180 * DEGREE_MULTIPLIER * PI;
    // Serial.print(radial_distance);
    // Serial.print(", ");
    return (radial_distance + actual_distance) / start_distance;
}

void movetoPoint(Point destination) {
    Vector localisation = processedValues.robot_position;
    double distance = (localisation - Vector::fromPoint(destination)).distance;
    movement.setConstantVelocity(Velocity::Constant{
        movement.applySigmoid(400, 250, (distance) / 20, 0.9)});
    movement.setMoveToPointDirection(
        Direction::MoveToPoint{localisation, destination});
}

int onLine = 0;
void receiveEspData(const byte *buf, size_t size) {
    // load payload
    if (size != sizeof(onLine)) {
        Serial.print("Invalid esp data, expect: ");
        Serial.print(sizeof(onLine));
        Serial.print(", got: ");
        Serial.print(size);
        Serial.println("");
        return;
    }

    memcpy(&onLine, buf, sizeof(onLine));
    Serial.print("On line: ");
    Serial.println(onLine);
}

void receiveTxData(const byte *buf, size_t size) {
    // load payload
    if (size != sizeof(processedValues)) {
        Serial.print("Invalid data, expect: ");
        Serial.print(sizeof(processedValues));
        Serial.print(", got: ");
        Serial.print(size);
        Serial.println("");
        return;
    }
    memcpy(&processedValues, buf, sizeof(processedValues));

    // Serial.print("Ball distance: ");
    // printDouble(Serial, processedValues.ball_relativeposition.distance, 3, 1);
    // Serial.println("");

}



void setup() {
    Serial.begin(115200); 
    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial5.begin(115200);

    TeensyTeensySerial.setStream(&Serial1);
    TeensyTeensySerial.setPacketHandler(&receiveTxData);

    ESPSerial.setStream(&Serial5);
    ESPSerial.setPacketHandler(&receiveEspData);

    movement.initialize();
}

void loop() {
    TeensyTeensySerial.update();
    ESPSerial.update();

    //global bearing 
    double bearing = 0; //for now
    movement.setBearingSettings(-300,300,2,0,0);

    // Case 1.0: robot encounters line, line rejection
    if (processedValues.onLine == 1){
        movement.setConstantDirection(Direction::Constant{180-processedValues.angleBisector});
        movement.setConstantVelocity(Velocity::Constant{500});
        movement.setConstantBearing(Bearing::Constant{bearing});
    }

    // Case 2.0: ball in catchment area, zoom towards goal
    else if (processedValues.is_ball_in_catchment == 1){
        //does yellow goal exist? If yes
        if (processedValues.yellowgoal_exists == 1){
            movement.setConstantDirection(Direction::Constant{processedValues.yellowgoal_relativeposition.angle});
            movement.setConstantVelocity(Velocity::Constant{400});
            movement.setConstantBearing(Bearing::Constant{bearing});
        }
        //if yellow goal does not exist, then move to point (0,120), which is opponent goal
        else if (processedValues.yellowgoal_exists == 0){
            //gg
            movetoPoint({0,120});

        }
    }
    // Case 3.0: ball exists but not in catchment area. Curve towards ball
    else if (processedValues.ballExists == 1 && processedValues.is_ball_in_catchment == 0){
        // curve towards ball
        movement.setConstantDirection(Direction::Constant{
                ballAngleOffset(processedValues.ball_relativeposition.distance,
                                processedValues.ball_relativeposition.angle) +
                                processedValues.ball_relativeposition.angle})
        movement.setconstantVelocity(
                Velocity::constant{movement.applySigmoid(
                500, 300,
                curveAroundBallMultiplier(
                    processedValues.ball_relativeposition.angle,
                    processedValues.ball_relativeposition.distance - 20,
                    70 - 20),
                2.5)});
        movement.setConstantBearing(Bearing::Constant{bearing});
    }
    // Case 4.0: ball does not exist. So move to default position
    else if (processedValues.ballExists == 0 && processedValues.is_ball_in_catchment == 0){
        movetoPoint({0,-20});
        movement.setConstantBearing(Bearing::Constant{bearing});
    }

    //drive code
    movement.drive(processedValues.robot_position);
}

    // Serial.print("On line: ");
    // Serial.print(onLine);
    // Serial.print(" | ");
    // Serial.print("Ball distance: ");
    // printDouble(Serial, processedValues.ball_relativeposition.distance, 3, 1);
    // // Serial.print(" | ");
    // // Serial.print("Ball angle: ");
    // // printDouble(Serial, processedValues.ball_relativeposition.angle, 3, 1);
    // Serial.print(" | ");
    // Serial.print("Bearing: ");
    // printDouble(Serial, processedValues.relativeBearing, 3, 1);
    // Serial.print(" | ");
    // Serial.print("Blue goal: ");
    // printDouble(Serial, processedValues.bluegoal_relativeposition.distance, 3, 1);
    // Serial.println("");



    
