#ifndef BALLPOSITION_H
#define BALLPOSITION_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <array>

#include "kalman.h"
#include "main.h"
#include "shared.h"
#include "util.h"
#include "vector.h"

class BallPosition {
   public:
    BallPosition(); // constructor

    Vector updatePosition();  // updates estimated ball position
    void updateSensorMeasurement(int x_cam, int y_cam);  // updates sensor measurements
    void updateConstants(int dt);  // updates kalman filter constants

   private:
    // number of states, measurements, and control inputs
    static const int n = 6;  // number of state variables
    static const int m = 2;  // number of measurements
    static const int _l = 2; // number of control inputs

    int dt = 0;               // time step
    int cameraVariance = 10;  // variance of camera measurement

    int x_camera = 0, y_camera = 0; // camera sensor values

    // kalman filter matrices
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);   // state transition matrix
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, _l);  // control input matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, n);   // measurement matrix
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);   // process noise covariance
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);   // measurement noise covariance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n); // identity matrix
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, n);   // error covariance
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n, m);   // kalman gain

    Eigen::VectorXd x_hat = Eigen::VectorXd::Zero(n);  // initial state estimate
    Eigen::VectorXd z = Eigen::VectorXd::Zero(m);      // measurement vector
    Eigen::VectorXd u = Eigen::VectorXd::Zero(_l);     // control input vector

    KalmanFilter kf;  // kalman filter instance
};

#endif  // BALLPOSITION_H