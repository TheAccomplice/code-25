#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>

#include <array>

#include "PacketSerial.h"
#include "kalman.h"
#include "shared.h"
#include "util.h"

// sensor fusion class for integrating multiple sensor inputs
class Sensorfusion {
  public:
    Sensorfusion(); // constructor
    Vector updateLocalisation(); // updates the robot's localization based on sensor data
    void updateSensorValues(int flpwm, int frpwm, int blpwm, int brpwm,
                            int frontTof, int backTof, int leftTof,
                            int rightTof, int x_camera, int y_camera); // updates sensor readings
    void updateConstants(double frontVariance, double backVariance,
                         double leftVariance, double rightVariance,
                         double x_camVariance, double y_camVariance); // updates sensor variances

  private:
    // n = number of states
    // m = number of measurements
    // l = number of control inputs
    int n = 4;
    int m = 6;
    int _l = 2;

    int dt = loopTimeinmicros(); // loop time in microseconds
    int tofVariance = 111115; // time-of-flight sensor variance
    int lidarVariance = 11115; // lidar sensor variance
    int cameraVariance = 20; // camera sensor variance

    // motor pwm values
    int flpwm, frpwm, blpwm, brpwm;

    // pwm multipliers for left and right movement vectors
    int leftVectorPWMmultiplier, rightVectorPWMmultiplier;
    
    // time-of-flight sensor readings
    int frontTof, backTof, leftTof, rightTof;
    
    // camera x and y position
    int x_camera, y_camera;

    // kalman filter matrices
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n); // state transition matrix
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, _l); // control input matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, n); // measurement matrix
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n); // process noise covariance matrix
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m); // measurement noise covariance matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n); // identity matrix
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n); // estimate covariance matrix
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n, m); // kalman gain matrix

    Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(n); // initial state estimate
    Eigen::VectorXd z = Eigen::VectorXd::Zero(m); // measurement vector
    Eigen::VectorXd u = Eigen::VectorXd::Zero(_l); // control input vector

    void initializeKalmanFilter(); // initializes the kalman filter

    KalmanFilter kf; // kalman filter instance
};

#endif // SENSORFUSION_H