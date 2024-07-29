/**
* Kalman filter header file.
*
* @author: Dhruv Shah, Hayk Martirosyan
* @date: 07/03/2018
*/

#include <eigen3/Eigen/Dense>

#pragma once

class KalmanFilter {

public:

    /**
    * Create a Kalman filter with the specified matrices.
    *   A - System dynamics matrix
    *   B - Input matrix
    *   C - Output matrix
    *   Q - Process noise covariance
    *   R - Measurement noise covariance
    *   P - Estimate error covariance
    */
    KalmanFilter(
        const Eigen::MatrixXf& A,
        const Eigen::MatrixXf& B,
        const Eigen::MatrixXf& C,
        const Eigen::MatrixXf& Q,
        const Eigen::MatrixXf& R,
        const Eigen::MatrixXf& P
    );

    /**
    * Initialize the filter with initial states as zero.
    */
    void init();

    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(const Eigen::VectorXf& x0);

    /**
    * Update the prediction based on control input.
    */
    void predict(const Eigen::VectorXf& u);

    /**
    * Update the estimated state based on measured values.
    */
    void update(const Eigen::VectorXf& y);

    /**
    * Update the dynamics matrix.
    */
    void update_dynamics(const Eigen::MatrixXf A);

    /**
    * Update the output matrix.
    */
    void update_output(const Eigen::MatrixXf C);

    /**
    * Return the current state.
    */
    Eigen::VectorXf state() { return x_hat; };

private:

    // Matrices for computation
    Eigen::MatrixXf A, B, C, Q, R, P, K, P0;

    // System dimensions
    int m, n, c;

    // Is the filter initialized?
    bool initialized = false;

    // n-size identity
    Eigen::MatrixXf I;

    // Estimated states
    Eigen::VectorXf x_hat;
};