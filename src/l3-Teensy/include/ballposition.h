#ifndef BALLPOSITION_H
#define BALLPOSITION_H

#include <Arduino.h>
#include "shared.h"
#include "util.h"

class BallPosition {
public:
    BallPosition();
    Vector updatePosition();
    void updateSensorMeasurement(int x_cam, int y_cam);
private:
    int x_camera, y_camera;
    Vector position;
};

#endif