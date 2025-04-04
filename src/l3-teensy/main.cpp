#include "PacketSerial.h"
#include "SPI.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "vector.h"
#include <Arduino.h>
#include <PacketSerial.h>
#include <Wire.h>
#include <iostream>
#include <math.h>
#include "kalman.h"
#include "sensorfusion.h"
#include "ballposition.h"
#include "config.h"

#define TEENSY


#define LDRPINCOUNT 36
#define RadiusofLDR 1.0F

DMAMEM Sensorfusion sensorfusion;
DMAMEM BallPosition ballposition;


PacketSerial CameraTeensySerial;
PacketSerial TeensyTeensySerial;
// PacketSerial LidarTeensySerial;
BNO08x bno;



SensorValues sensorValues;
ProcessedValues processedValues;

Point ball_last_position_relative_to_robot;

FLASHMEM void receiveLidarTxData(const byte *buf, size_t size) {
    // load payload
    LidarTxPayload payload;
    // if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));
    for (int i = 0; i < 4; i++) {
        sensorValues.lidardist[i] = payload.lidarTxData.distance[i];
    }
    return;

}

void receiveCameraTxData(const byte *buf, size_t size) {
    // load payload
    CameraTxPayload payload;
    if (size != sizeof(payload)) {
        Serial.print("Invalid data, size: ");
        Serial.print(size);
        Serial.println("");
        return;
    }

    memcpy(&payload, buf, sizeof(payload));
#ifdef DEBUG
    // Serial.print("Size: ");
    // Serial.print(", [2]: ");
    // printDouble(Serial, payload.cameraTxData.values[2], 3, 1);
    // Serial.print(", [3]: ");
    // printDouble(Serial, payload.cameraTxData.values[3], 3, 1);
    // // Serial.println("");
#endif

    #ifdef YELLOW_GOAL_ATTACK
    sensorValues.bluegoal_relativeposition.angle =
        payload.cameraTxData.values[0];
    sensorValues.bluegoal_relativeposition.distance =
        payload.cameraTxData.values[1];
    sensorValues.yellowgoal_relativeposition.angle =
        payload.cameraTxData.values[2];
    sensorValues.yellowgoal_relativeposition.distance =
        payload.cameraTxData.values[3];
    #endif
    #ifdef BLUE_GOAL_ATTACK
    sensorValues.yellowgoal_relativeposition.angle =
        payload.cameraTxData.values[0];
    sensorValues.yellowgoal_relativeposition.distance =
        payload.cameraTxData.values[1];
    sensorValues.bluegoal_relativeposition.angle =
        payload.cameraTxData.values[2];
    sensorValues.bluegoal_relativeposition.distance =
        payload.cameraTxData.values[3];
    #endif
    sensorValues.ball_relativeposition.angle = payload.cameraTxData.values[4];
    sensorValues.ball_relativeposition.distance =
        payload.cameraTxData.values[5];
    //sensorValues.ball_relativeposition.distance = ballMirrorMapping(sensorValues.ball_relativeposition.distance);
    sensorValues.ball_relativeposition.distance = frontMirrorMapping(sensorValues.ball_relativeposition.distance);
    sensorValues.bluegoal_relativeposition.distance = frontMirrorMapping(sensorValues.bluegoal_relativeposition.distance);
    sensorValues.yellowgoal_relativeposition.distance = frontMirrorMapping(sensorValues.yellowgoal_relativeposition.distance);
    return;
}

void getBNOreading() {
    bno.enableGyroIntegratedRotationVector();
    if (bno.getSensorEvent() == true) {
        if (bno.getSensorEventID() ==
            SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR) {
            sensorValues.relativeBearing =
                bno.getGyroIntegratedRVK()
                * 180.0 * 0.8; // Convert yaw / heading to degree
        }
    }
}

void setReports(void) {

    // if (bno.enableGyroIntegratedRotationVector()== true) {
    //     // Serial.println(F("Gryo Integrated Rotation vector enabled"));
    //     // Serial.println(F("Output in form i, j, k, real, gyroX, gyroY,
    //     // gyroZ"));
    // } else {
    //     // Serial.println("Could not enable gyro integrated rotation vector");
    // }
}

void predict_ball_in_catchment(){
    if (ball_last_position_relative_to_robot.x < 10 && 
        ball_last_position_relative_to_robot.x > -10 &&
        ball_last_position_relative_to_robot.y < 40 &&
        ball_last_position_relative_to_robot.y > 2 &&
        processedValues.ballExists == 0){
            processedValues.ball_in_catchment = 1;
        }
    else if (processedValues.ball_in_catchment == 1 && processedValues.ballExists == 0) {
        processedValues.ball_in_catchment = 1;
    }
    else {
        processedValues.ball_in_catchment = 0;
    }
}

FLASHMEM Vector localize() {
    if ((sensorValues.yellowgoal_relativeposition.distance < 90 &&
         processedValues.yellowgoal_exists == 1) ||
        (processedValues.yellowgoal_exists == 1 &&
         processedValues.bluegoal_exists == 0)) {
        sensorfusion.updateSensorValues(
            0, 0,
            0, 0,
            sensorValues.lidardist[0], sensorValues.lidardist[2],
            sensorValues.lidardist[3], sensorValues.lidardist[1],
            localizeWithOffensiveGoal().x(), localizeWithOffensiveGoal().y());
        Vector localisation = sensorfusion.updateLocalisation();
        // Serial.print("f");
        return localisation;
    } else if ((sensorValues.bluegoal_relativeposition.distance < 90 &&
                processedValues.bluegoal_exists == 1) ||
               (processedValues.bluegoal_exists == 1 &&
                processedValues.yellowgoal_exists == 0)) {
        sensorfusion.updateSensorValues(
            0, 0,
            0, 0,
            sensorValues.lidardist[0], sensorValues.lidardist[2],
            sensorValues.lidardist[3], sensorValues.lidardist[1],
            localizeWithDefensiveGoal().x(), localizeWithDefensiveGoal().y());
        Vector localisation = sensorfusion.updateLocalisation();
        // Serial.print("b");
        return localisation;
    } else {
        sensorfusion.updateSensorValues(
            0, 0,
            0, 0,
            sensorValues.lidardist[0], sensorValues.lidardist[2],
            sensorValues.lidardist[3], sensorValues.lidardist[1],
            localizeWithBothGoals().x(), localizeWithBothGoals().y());
        Vector localisation = sensorfusion.updateLocalisation();
        // Serial.print("fb");
        return localisation;
    }
    // sensorfusion.updateSensorValues(movement.getmotorValues()[0],movement.getmotorValues()[1],movement.getmotorValues()[2],
    //                                   movement.getmotorValues()[3], 0,
    //                                   sensorValues.lidardist[3], 0, 0,
    //                                   localizeWithDefensiveGoal().x(),
    //                                   localizeWithDefensiveGoal().y());
}

void verifyingObjectExistance() {
    processedValues.ballExists =
        (processedValues.ball_relativeposition.distance == 0) ? 0 : 1;
    processedValues.bluegoal_exists =
        (sensorValues.bluegoal_relativeposition.distance == 0) ? 0 : 1;
    processedValues.yellowgoal_exists =
        (sensorValues.yellowgoal_relativeposition.distance == 0) ? 0 : 1;

}


void setup() {
    Serial1.begin(115200);
    Serial3.begin(115200);
    Serial2.begin(115200);
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(100000);
    bno.begin(0x4A, Wire);
    CameraTeensySerial.setStream(&Serial1);
    CameraTeensySerial.setPacketHandler(&receiveCameraTxData);

    LidarTeensySerial.begin(&Serial2);
    LidarTeensySerial.setPacketHandler(&receiveLidarTxData);

    TeensyTeensySerial.begin(&Serial3);

    setReports();

    analogWriteResolution(10);

    // byte buf[sizeof(l1TxPayload)];
    // memcpy(buf,&SAMDlinedata, sizeof(SAMDlinedata));
    // L1Serial.send(buf,sizeof(buf));
}

int counter = 0;
double frontVariance = 2;
double backVariance = 2;
double leftVariance = 2;
double rightVariance = 2;

#ifdef DEBUG
#ifdef real
void loop() {
    double dt = loopTimeinMillis();
    CameraTeensySerial.update();

    // unsigned char tmp[256];
    // int count = Serial1.readBytes(tmp, 256);
    // static int j = 0;
    // for (int i=0; i<count; i++,j++){
    //     if (tmp[i] == 0){
    //         Serial.println("");
    //         Serial.println("");
    //         j=0;
    //         continue;
    //     }
    //     if (tmp[i] < 16)
    //         Serial.print(" ");
    //     Serial.print(tmp[i], HEX);
    //     Serial.print(" ");
    //     if (j%16 == 0){
    //         Serial.println("");
    //     }   
    // } 

    // LidarTeensySerial.update();
    setReports();
    getBNOreading();
    // Serial.print("Relative Bearing: ");
    // Serial.print(sensorValues.relativeBearing);
    // Serial.print(" | ");

    processedValues.bluegoal_relativeposition = sensorValues.bluegoal_relativeposition;
    processedValues.bluegoal_relativeposition.distance = sensorValues.bluegoal_relativeposition.distance;
    processedValues.yellowgoal_relativeposition = sensorValues.yellowgoal_relativeposition;
    processedValues.yellowgoal_relativeposition.distance = sensorValues.yellowgoal_relativeposition.distance;
    processedValues.relativeBearing = -sensorValues.relativeBearing;

    //setup
    processLidars();
    // Serial.print("Bearing: ");
    // Serial.println(processedValues.relativeBearing);

    (processedValues.lidarConfidence[0] == 1) ? frontVariance = 3 : frontVariance = 400;
    (processedValues.lidarConfidence[1] == 1) ? rightVariance = 3 : rightVariance = 400;
    (processedValues.lidarConfidence[2] == 1) ? backVariance = 3 : backVariance = 400;
    (processedValues.lidarConfidence[3] == 1) ? leftVariance = 0 : leftVariance = 400;

    sensorfusion.updateConstants(frontVariance, backVariance, leftVariance,
                                rightVariance, 10, 15);

    // kalman commented out
    // ballposition.updateConstants(dt / 1000);
    // ballposition.updateSensorMeasurement(
    //     sensorValues.ball_relativeposition.x(),
    //     sensorValues.ball_relativeposition.y());
    // processedValues.ball_relativeposition = ballposition.updatePosition();

    Vector robotPosition = localize();
    processedValues.robot_position = {robotPosition.x(),robotPosition.y()};

    //must be below updatesensormesaurement
    verifyingObjectExistance();
    predict_ball_in_catchment();

    #ifdef DEBUG
    // Serial.print(" | bearing: ");
    // printDouble(Serial, processedValues.relativeBearing, 3, 1);
    // Serial.print(" | frontLidar: ");
    // printDouble(Serial, processedValues.lidarDistance[0], 3, 1);
    // Serial.print(" | rightLidar: ");
    // printDouble(Serial, processedValues.lidarDistance[1], 3, 1);
    // Serial.print(" | backLidar: ");
    // printDouble(Serial, processedValues.lidarDistance[2], 3, 1);
    // Serial.print(" | leftLidar: ");
    // printDouble(Serial, processedValues.lidarDistance[3], 3, 1);

    // Serial.print(" | frontLidarConf: ");
    // printDouble(Serial, processedValues.lidarConfidence[0], 3, 1);
    // Serial.print(" | rightLidarConf: ");
    // printDouble(Serial, processedValues.lidarConfidence[1], 3, 1);
    // Serial.print(" | backLidarConf: ");
    // printDouble(Serial, processedValues.lidarConfidence[2], 3, 1);
    // Serial.print(" | leftLidarConf: ");
    // printDouble(Serial, processedValues.lidarConfidence[3], 3, 1);
        
    // Serial.print(" | attackGoalAngle: ");
    // printDouble(Serial, processedValues.yellowgoal_relativeposition.angle, 3, 1);
    // Serial.print(" | attackGoalDist: ");
    // printDouble(Serial, processedValues.yellowgoal_relativeposition.distance, 3,
    //             1);
    // Serial.print(" | defenceGoalAngle: ");
    // printDouble(Serial, processedValues.bluegoal_relativeposition.angle, 3, 1);
    // Serial.print(" | defenceGoalDist: ");
    // printDouble(Serial, processedValues.bluegoal_relativeposition.distance, 3, 1);
    Serial.print(" | ballAngle: ");
    printDouble(Serial, sensorValues.ball_relativeposition.angle, 3, 1);
    Serial.print(" | ballDist: ");
    printDouble(Serial, sensorValues.ball_relativeposition.distance, 3, 1);
    // processed Values
    Serial.print(" | ballExistence: ");
    printDouble(Serial, processedValues.ballExists, 1, 1);
    // Serial.print(" | attackGoal Existence: ");
    // printDouble(Serial, processedValues.yellowgoal_exists, 1, 1);
    // Serial.print(" | defenceGoal Existence: ");
    // printDouble(Serial, processedValues.bluegoal_exists, 1, 1);
    // // Location
    // Serial.print(" | X_position: ");
    // printDouble(Serial, processedValues.robot_position.x, 3, 1);
    // Serial.print(" | Y_position: ");
    // printDouble(Serial, processedValues.robot_position.y, 3, 1);

    // Prediction Parameters
    Serial.print(" | ball in catchment: ");
    printDouble(Serial, processedValues.ball_in_catchment, 1, 1);
    Serial.print(" | x_ball_postion to robot: ");
    printDouble(Serial, ball_last_position_relative_to_robot.x, 1, 1);
    Serial.print(" | y_ball postion to robot: ");
    printDouble(Serial, ball_last_position_relative_to_robot.y, 1, 1);
    

    Serial.println("");
    #endif

    // delay(1000); //this is the problem

    

    byte buf[sizeof(teensytoTeensyTxPayload)];
    memcpy(buf, &processedValues, sizeof(processedValues));
    TeensyTeensySerial.send(buf, sizeof(buf));

    // if (processedValues.ballExists == 1 ) {
    //     if (sensorValues.ball_relativeposition.distance == 0) {
    //         Serial.println("invalid");
    //     } else {
    //         ball_last_position_relative_to_robot = {sensorValues.ball_relativeposition.x(),
    //         sensorValues.ball_relativeposition.y()};
    //         Serial.print("yay");
    //     }
    // };

    
    counter++;
}
#endif


#ifdef test
    double dt = loopTimeinMillis();
    CameraTeensySerial.update();

    LidarTeensySerial.update();
    setReports();
    getBNOreading();
    // Serial.print("Relative Bearing: ");
    // Serial.print(sensorValues.relativeBearing);
    // Serial.print(" | ");

    processedValues.bluegoal_relativeposition = sensorValues.bluegoal_relativeposition;
    processedValues.bluegoal_relativeposition.distance = sensorValues.bluegoal_relativeposition.distance;
    processedValues.yellowgoal_relativeposition = sensorValues.yellowgoal_relativeposition;
    processedValues.yellowgoal_relativeposition.distance = sensorValues.yellowgoal_relativeposition.distance;
    processedValues.relativeBearing = -sensorValues.relativeBearing;

    //setup
    processLidars();
    // Serial.print("Bearing: ");
    // Serial.println(processedValues.relativeBearing);

    (processedValues.lidarConfidence[0] == 1) ? frontVariance = 3 : frontVariance = 400;
    (processedValues.lidarConfidence[1] == 1) ? rightVariance = 3 : rightVariance = 400;
    (processedValues.lidarConfidence[2] == 1) ? backVariance = 3 : backVariance = 400;
    (processedValues.lidarConfidence[3] == 1) ? leftVariance = 0 : leftVariance = 400;

    sensorfusion.updateConstants(frontVariance, backVariance, leftVariance,
                                rightVariance, 10, 15);

    // kalman commented out
    // ballposition.updateConstants(dt / 1000);
    // ballposition.updateSensorMeasurement(
    //     sensorValues.ball_relativeposition.x(),
    //     sensorValues.ball_relativeposition.y());
    // processedValues.ball_relativeposition = ballposition.updatePosition();

    Vector robotPosition = localize();
    processedValues.robot_position = {robotPosition.x(),robotPosition.y()};

    //must be below updatesensormesaurement
    verifyingObjectExistance();
    predict_ball_in_catchment();

    #ifdef DEBUG
    // Serial.print(" | bearing: ");
    // printDouble(Serial, processedValues.relativeBearing, 3, 1);
    // Serial.print(" | frontLidar: ");
    // printDouble(Serial, processedValues.lidarDistance[0], 3, 1);
    // Serial.print(" | rightLidar: ");
    // printDouble(Serial, processedValues.lidarDistance[1], 3, 1);
    // Serial.print(" | backLidar: ");
    // printDouble(Serial, processedValues.lidarDistance[2], 3, 1);
    // Serial.print(" | leftLidar: ");
    // printDouble(Serial, processedValues.lidarDistance[3], 3, 1);

    // Serial.print(" | frontLidarConf: ");
    // printDouble(Serial, processedValues.lidarConfidence[0], 3, 1);
    // Serial.print(" | rightLidarConf: ");
    // printDouble(Serial, processedValues.lidarConfidence[1], 3, 1);
    // Serial.print(" | backLidarConf: ");
    // printDouble(Serial, processedValues.lidarConfidence[2], 3, 1);
    // Serial.print(" | leftLidarConf: ");
    // printDouble(Serial, processedValues.lidarConfidence[3], 3, 1);
        
    // Serial.print(" | attackGoalAngle: ");
    // printDouble(Serial, processedValues.yellowgoal_relativeposition.angle, 3, 1);
    // Serial.print(" | attackGoalDist: ");
    // printDouble(Serial, processedValues.yellowgoal_relativeposition.distance, 3,
    //             1);
    // Serial.print(" | defenceGoalAngle: ");
    // printDouble(Serial, processedValues.bluegoal_relativeposition.angle, 3, 1);
    // Serial.print(" | defenceGoalDist: ");
    // printDouble(Serial, processedValues.bluegoal_relativeposition.distance, 3, 1);
    Serial.print(" | ballAngle: ");
    printDouble(Serial, sensorValues.ball_relativeposition.angle, 3, 1);
    Serial.print(" | ballDist: ");
    printDouble(Serial, sensorValues.ball_relativeposition.distance, 3, 1);
    // processed Values
    Serial.print(" | ballExistence: ");
    printDouble(Serial, processedValues.ballExists, 1, 1);
    // Serial.print(" | attackGoal Existence: ");
    // printDouble(Serial, processedValues.yellowgoal_exists, 1, 1);
    // Serial.print(" | defenceGoal Existence: ");
    // printDouble(Serial, processedValues.bluegoal_exists, 1, 1);
    // // Location
    // Serial.print(" | X_position: ");
    // printDouble(Serial, processedValues.robot_position.x, 3, 1);
    // Serial.print(" | Y_position: ");
    // printDouble(Serial, processedValues.robot_position.y, 3, 1);

    // Prediction Parameters
    Serial.print(" | ball in catchment: ");
    printDouble(Serial, processedValues.ball_in_catchment, 1, 1);
    Serial.print(" | x_ball_postion to robot: ");
    printDouble(Serial, ball_last_position_relative_to_robot.x, 1, 1);
    Serial.print(" | y_ball postion to robot: ");
    printDouble(Serial, ball_last_position_relative_to_robot.y, 1, 1);
    

    Serial.println("");
    #endif

    // delay(1000); //this is the problem

    

    byte buf[sizeof(teensytoTeensyTxPayload)];
    memcpy(buf, &processedValues, sizeof(processedValues));
    TeensyTeensySerial.send(buf, sizeof(buf));

    // if (processedValues.ballExists == 1 ) {
    //     if (sensorValues.ball_relativeposition.distance == 0) {
    //         Serial.println("invalid");
    //     } else {
    //         ball_last_position_relative_to_robot = {sensorValues.ball_relativeposition.x(),
    //         sensorValues.ball_relativeposition.y()};
    //         Serial.print("yay");
    //     }
    // };

    
    counter++;
#endif