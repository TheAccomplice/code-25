#include "sensorfusion.h"

SensorFusion::SensorFusion() : position(0, 0) {}

void SensorFusion::updateSensorValues(int fl, int fr, int bl, int br,
                                      int front, int back, int left, int right,
                                      int x_cam, int y_cam) {
    flpwm = fl; frpwm = fr; blpwm = bl; brpwm = br;
    frontTof = front; backTof = back; leftTof = left; rightTof = right;
    x_camera = x_cam; y_camera = y_cam;
}

Vector SensorFusion::updateLocalisation() {
    float x_pos = ((91 - rightTof) + (leftTof - 91)) / 2.0 + x_camera;
    float y_pos = ((121.5 - frontTof) + (backTof - 121.5)) / 2.0 + y_camera;
    position.x = x_pos;
    position.y = y_pos;
    return position;
}