#include "kalman.h"

// constructor: initializes the kalman filter with matrices A, B, Q, R, I, and H
KalmanFilter::KalmanFilter(Eigen::MatrixXd A, Eigen::MatrixXd B,
                           Eigen::MatrixXd Q, Eigen::MatrixXd R,
                           Eigen::MatrixXd I, Eigen::MatrixXd H)
    : _A(A), _B(B), _Q(Q), _R(R), _I(I), _H(H), n(A.rows()), m(H.rows()), _l(B.cols()) {
    // x_hat and P(n, n) are necessary because:
    // - x_hat represents the estimated state of the system (position, velocity, etc.)
    // - P(n, n) is the state covariance matrix, representing the uncertainty in the estimate
}

// updates the kalman filter matrices
void KalmanFilter::updateConstants(Eigen::MatrixXd A, Eigen::MatrixXd B,
                                   Eigen::MatrixXd Q, Eigen::MatrixXd R,
                                   Eigen::MatrixXd I, Eigen::MatrixXd H) {
    _A = A;
    _B = B;
    _Q = Q;
    _R = R;
    _I = I;
    _H = H;
}

// initializes the kalman filter state, covariance, and kalman gain
void KalmanFilter::initialize(Eigen::VectorXd x_hat, Eigen::MatrixXd P,
                              Eigen::MatrixXd K) {
    _x_hat = x_hat;
    _P = P;
    _K = K;
}

// utility function to print a matrix
void printMatrix(Eigen::MatrixXd x, int row, int col) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            Serial.print(x(i, j));
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();
}

// utility function to print a vector
void printVector(Eigen::VectorXd x, int row) {
    for (int i = 0; i < row; i++) {
        Serial.print(x(i));
        Serial.print(" ");
        Serial.println();
    }
    Serial.println();
}

// kalman filter prediction step
void KalmanFilter::predict(Eigen::VectorXd u) {
    _x_hat = _A * _x_hat + _B * u;  // a priori state estimate

    // update the error covariance matrix
    _P = _A * _P * _A.transpose() + _Q;
}

// kalman filter correction step
void KalmanFilter::correction(Eigen::VectorXd z) {
    _z = z;

    // compute the kalman gain
    _K = _P * _H.transpose() * (_H * _P * _H.transpose() + _R).inverse();

    // update state estimate with measurement correction
    _x_hat = _x_hat + _K * (_z - _H * _x_hat);

    // update covariance estimate
    _P = (_I - _K * _H) * _P;
}

// returns the updated state estimate
Eigen::MatrixXd KalmanFilter::updateState() {
    return _x_hat;
}