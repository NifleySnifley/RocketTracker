/**
* Implementation of KalmanFilter class.
*
* @author: Dhruv Shah, Hayk Martirosyan
* @date: 07/03/2018
*/

#include <iostream>
#include <iostream>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXf& A,
    const Eigen::MatrixXf& B,
    const Eigen::MatrixXf& C,
    const Eigen::MatrixXf& Q,
    const Eigen::MatrixXf& R,
    const Eigen::MatrixXf& P)
    : A(A), B(B), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), c(B.cols()), initialized(false),
    I(n, n), x_hat(n) {
    I.setIdentity();
}

void KalmanFilter::init(const Eigen::VectorXf& x0) {

    x_hat = x0;
    P = P0;
    initialized = true;
}

void KalmanFilter::init() {

    x_hat.setZero();
    P = P0;
    initialized = true;
}

void KalmanFilter::predict(const Eigen::VectorXf& u) {

    if (!initialized) {
        std::cout << "Filter is not initialized! Initializing with trivial state.";
        init();
    }

    x_hat = A * x_hat + B * u;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXf& y) {

    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat += K * (y - C * x_hat);
    P = (I - K * C) * P;
}

void KalmanFilter::update_dynamics(const Eigen::MatrixXf A) {

    this->A = A;
}

void KalmanFilter::update_output(const Eigen::MatrixXf C) {

    this->C = C;
}