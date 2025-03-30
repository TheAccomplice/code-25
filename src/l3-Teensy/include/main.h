#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <array>
#include "PacketSerial.h"
#include "config.h"
#include "util.h"
#include "vector.h"

// structure to store processed sensor values
struct ProcessedValues {
    Vector ball_relative_position;          // position of the ball relative to the robot
    Vector yellow_goal_relative_position;   // position of the yellow goal relative to the robot
    Vector blue_goal_relative_position;     // position of the blue goal relative to the robot
    int ball_exists = 0;                     // flag to indicate if ball is detected
    int yellow_goal_exists = 0;              // flag to indicate if yellow goal is detected
    int blue_goal_exists = 0;                // flag to indicate if blue goal is detected
    int lidar_distance[4];                   // array to store lidar distances
    double lidar_confidence[4] = {};              // array to store lidar confidence levels
    int relative_bearing;                    // robot's relative bearing
    Point robot_position;                     // position of the robot
};

// structure to store raw sensor values
struct SensorValues {
    int relative_bearing;                    // robot's relative bearing
    Vector yellow_goal_relative_position;    // position of the yellow goal relative to the robot
    Vector blue_goal_relative_position;      // position of the blue goal relative to the robot
    Vector ball_relative_position;           // position of the ball relative to the robot
    int lidar_distance[4];                   // array to store lidar distances
};

// global sensor value structures
extern SensorValues sensorValues;
extern ProcessedValues processedValues;

// structure for data transmission between teensy boards
struct TeensyToTeensyTxPayload {
    ProcessedValues processed_values;
};

// structure for camera transmission data
struct CameraTxData {
    double values[6]; // stores camera data values
};

// structure for camera transmission payload
struct CameraTxPayload {
    CameraTxData camera_tx_data;
};

// structure for lidar transmission data
struct LidarTxData {
    int distance[4]; // stores lidar distance readings
};

// structure for lidar transmission payload
struct LidarTxPayload {
    LidarTxData lidar_tx_data;
};

// function prototypes
void verifyingObjectExistence();            // verifies existence of detected objects
void processLidars();                        // processes lidar data
Vector localizeWithOffensiveGoal();          // localizes robot using the offensive goal
Vector localizeWithDefensiveGoal();          // localizes robot using the defensive goal
Vector localizeWithBothGoals();              // localizes robot using both goals
double frontMirrorMapping(double distance);  // maps front mirror distance
double ballMirrorMapping(double distance);   // maps ball mirror distance

#endif // MAIN_H