#include "ballposition.h"

BallPosition::BallPosition() : position(0, 0), x_camera(0), y_camera(0) {}

void BallPosition::updateSensorMeasurement(int x_cam, int y_cam) {
    x_camera = x_cam;
    y_camera = y_cam;
}

Vector BallPosition::updatePosition() {
    position.x = x_camera;
    position.y = y_camera;
    return position;
}