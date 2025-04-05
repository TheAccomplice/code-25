#ifndef MAIN_H
#define MAIN_H
#include <Arduino.h>
#include <ArduinoEigenDense.h>

#include <array>

#include "PacketSerial.h"
#include "config.h"
#include "util.h"
#include "vector.h"
#include "shared.h"

// struct ProcessedValues {
//     //relative to robot
//     Vector ball_relativeposition;
//     Vector yellowgoal_relativeposition;
//     Vector bluegoal_relativeposition;
//     //relative to field, not used?
//     Vector ball_actualposition;
//     Vector yellowgoal_actualposition;
//     Vector bluegoal_actualposition;
//     Vector robot_position;

//     int ballExists = 0;
//     int ball_in_catchment = 0;
//     int yellowgoal_exists = 0;
//     int bluegoal_exists = 0;
//     int lidarDistance[4];
//     double lidarConfidence[4];
//     double bearing_relative_to_field;
//     int is_ball_in_catchment = 0;
//     int relativeBearing;
//     int onLine = 0;
//     float angleBisector;
//     float depthinLine;
//     double depth_in_line;
//     // double past_true_x_left = 0;
//     // double past_true_x_right = 0;
//     // double past_true_y_front = 0;
//     // double past_true_y_back = 0;
// };

struct SensorValues {
    int relativeBearing;
    Vector yellowgoal_relativeposition;
    Vector bluegoal_relativeposition;
    double ball_dist;
    Vector ball_relativeposition;
    int lidardist[4];
};


extern SensorValues sensorValues;
extern ProcessedValues processedValues;


struct teensytoTeensyTxPayload {
    ProcessedValues processedValues;
};

struct CameraTxData {
    double values[6];
};

struct CameraTxPayload {
    CameraTxData cameraTxData;
};

struct LidarTxData {
    int distance[4];
};

struct LidarTxPayload {
    LidarTxData lidarTxData;
};





//subroutines
void verifyingObjectExistance();
void processLidars();
Vector localizeWithOffensiveGoal();
Vector localizeWithDefensiveGoal();
Vector localizeWithBothGoals();
double frontMirrorMapping(double distance);
double ballMirrorMapping(double distance);
#endif