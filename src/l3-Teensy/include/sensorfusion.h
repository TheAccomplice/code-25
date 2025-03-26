#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include <Arduino.h>
#include "shared.h"
#include "util.h"

class SensorFusion {
public:
    SensorFusion();
    Vector updateLocalisation();
    void updateSensorValues(int flpwm, int frpwm, int blpwm, int brpwm,
                            int frontTof, int backTof, int leftTof,
                            int rightTof, int x_camera, int y_camera);
private:
    int flpwm, frpwm, blpwm, brpwm;
    int frontTof, backTof, leftTof, rightTof;
    int x_camera, y_camera;
    Vector position;
};

#endif