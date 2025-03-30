#ifndef KALMAN_H
#define KALMAN_H

#include <ArduinoEigenDense.h>
#undef _B
#undef _P
class KalmanFilter {
  public:
    // constructor initializes kalman filter with given matrices
    KalmanFilter(const Eigen::MatrixXd& A,      // system dynamic coefficient matrix
                 const Eigen::MatrixXd& B,      // control input coefficient matrix
                 const Eigen::MatrixXd& Q,      // process noise covariance
                 const Eigen::MatrixXd& R,      // measurement noise covariance
                 const Eigen::MatrixXd& I,      // identity matrix
                 const Eigen::MatrixXd& H       // measurement coefficient matrix
    );

    // updates kalman filter matrices
    void updateConstants(const Eigen::MatrixXd& A, 
                         const Eigen::MatrixXd& B, 
                         const Eigen::MatrixXd& Q, 
                         const Eigen::MatrixXd& R, 
                         const Eigen::MatrixXd& I, 
                         const Eigen::MatrixXd& H);

    // initializes state estimate and covariance matrix
    void initialize(const Eigen::VectorXd& x_hat, 
                    const Eigen::MatrixXd& P, 
                    const Eigen::MatrixXd& K);

    // prediction step of kalman filter
    void predict(const Eigen::VectorXd& u);

    // correction step of kalman filter
    void correction(const Eigen::VectorXd& z);

    // returns the updated state estimate
    Eigen::VectorXd updateState() const;

  private:
    // system matrices
    Eigen::MatrixXd _A, _B, _Q, _R, _I, _H;

    // kalman filter variables
    Eigen::MatrixXd _P; // apriori and aposteriori estimate error
    Eigen::MatrixXd _K; // kalman gain

    // matrix dimensions
    int _n, _m, _l;

    // measurement and state variables
    Eigen::VectorXd _u; // control input
    Eigen::VectorXd _z; // measurement
    Eigen::VectorXd _x_hat; // state estimate
};

#endif // KALMAN_H