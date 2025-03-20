#include "SensorFusion.h"

// constructor: initializes the kalman filter with predefined matrices
SensorFusion::SensorFusion() : kf(A, B, Q, R, I, H) {
    kf.updateConstants(A, B, Q, R, I, H);
    kf.initialize(x_hat, P, K);
}

// updates the kalman filter matrices based on sensor variances
void SensorFusion::updateConstants(double frontVariance, double backVariance,
                                   double leftVariance, double rightVariance,
                                   double x_camVariance, double y_camVariance) {
    // state transition matrix (A)
    A << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;

    /*
    assuming max rpm is 900, wheel diameter is 52mm
    max speed = 240 cm/s
    input assumed to be in fl - br, fr - bl format
    angle between wheel and x-axis = 34 degrees
    */

    // control input matrix (B)
    B << sin(56 * M_PI / 180) * dt, sin(56 * M_PI / 180) * dt,
         sin(34 * M_PI / 180) * dt, sin(34 * M_PI / 180) * dt,
         sin(56 * M_PI / 180),     sin(56 * M_PI / 180),
         sin(34 * M_PI / 180),     sin(34 * M_PI / 180);

    // measurement matrix (H)
    H << 1, 0, 0, 0,
         1, 0, 0, 0,
         0, 1, 0, 0,
         0, 1, 0, 0,
         1, 0, 0, 0,
         0, 1, 0, 0;

    // process noise covariance matrix (Q)
    Q << 20, 0,  0,  0,
         0,  20, 0,  0,
         0,  0,  20, 0,
         0,  0,  0,  20;

    // measurement noise covariance matrix (R)
    R << rightVariance, 0, 0, 0, 0, 0,
         0, leftVariance, 0, 0, 0, 0,
         0, 0, frontVariance, 0, 0, 0,
         0, 0, 0, backVariance, 0, 0,
         0, 0, 0, 0, x_camVariance, 0,
         0, 0, 0, 0, 0, y_camVariance;

    // covariance matrix (P)
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // identity matrix (I)
    I << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // update kalman filter with new constants
    kf.updateConstants(A, B, Q, R, I, H);
}

// updates sensor values and performs kalman filter prediction/correction
void SensorFusion::updateSensorValues(int flpwm, int frpwm, int blpwm,
                                      int brpwm, int frontTof, int backTof,
                                      int leftTof, int rightTof, int x_camera,
                                      int y_camera) {
    // control input (u) - currently set to zero
    u << 0, 0;

    // initialize state estimate (x_hat)
    x_hat << 0, 0, 0, 0;

    // measurement vector (z)
    z << (91 - rightTof),
         (leftTof - 91),
         (121.5 - frontTof),
         (backTof - 121.5),
         x_camera,
         y_camera;

    // kalman filter prediction and correction
    kf.predict(u);
    kf.correction(z);
}

// updates and returns the estimated position as a vector
Vector SensorFusion::updateLocalisation() {
    Eigen::VectorXd state = kf.updateState();

    Point location;
    location.x = state(0);
    location.y = state(1);

    return Vector::fromPoint(location);
}

