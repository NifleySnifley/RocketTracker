#include "alt_filter.h"
#include "Fusion.h"

#include <eigen3/Eigen/Eigen>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "kalman.hpp"
#include "esp_log.h"

void print_matrix(Eigen::MatrixXf matrix, const char* label) {
    std::stringstream ss;
    ss << matrix;
    ESP_LOGI("ALT-FILTER", "Matrix %s = %s", label, ss.str().c_str());
}

void discretize_ab(Eigen::MatrixXf* A, Eigen::MatrixXf* B, float dt) {
    int states = A->cols();
    int inputs = B->cols();

    Eigen::MatrixXf block(states + inputs, states + inputs);
    block.fill(0.0f);

    block.block(0, 0, A->rows(), A->cols()) = *A;
    block.block(0, A->cols(), B->rows(), B->cols()) = *B;

    // (block * dt).exp().evalTo(&block);

    *A = (block * dt).exp().block(0, 0, A->rows(), A->cols());
    *B = (block * dt).exp().block(0, A->cols(), B->rows(), B->cols());
}

void altimetry_filter_init(altimetry_filter_t* filter, float dt, float vertical_acceleration_stdev, float altimetry_stdev) {
    KalmanFilter* Kf = (KalmanFilter*)filter->Kf;

    int n = 2; // 2 States
    int m = 1; // 1 Measurement
    int c = 1; // 1 Control input

    Eigen::MatrixXf A(n, n); // System dynamics matrix
    Eigen::MatrixXf B(n, c); // Input control matrix
    Eigen::MatrixXf C(m, n); // Output matrix
    Eigen::MatrixXf Q(n, n); // Process noise covariance
    Eigen::MatrixXf R(m, m); // Measurement noise covariance
    Eigen::MatrixXf P(n, n); // Estimate error covariance
    A.fill(0.0f);
    B.fill(0.0f);
    C.fill(0.0f);
    Q.fill(0.0f);
    R.fill(0.0f);
    P.fill(0.0f);

    A << 0.0f, 1.0f, 0.0f, 0.0f;
    // Discretize!
    // A = (A * dt).exp();

    B << 0.0f, 1.0f;

    C << 1.0f, 0.0f;

    Q.diagonal().fill(vertical_acceleration_stdev * vertical_acceleration_stdev);
    R.diagonal().fill(altimetry_stdev * altimetry_stdev);

    /*
    numeric_matrix<2, 2> Q = numeric_matrix<2, 2>::diagonals(1.3940354099852605e-05);
    numeric_matrix<1, 1> R = numeric_matrix<1, 1>::diagonals(0.06322944764207783);
    */

    discretize_ab(&A, &B, dt);

    P = Q;

    print_matrix(A, "A");
    print_matrix(B, "B");
    print_matrix(C, "C");
    print_matrix(Q, "Q");
    print_matrix(R, "R");
    print_matrix(P, "P");
    // B

    filter->Kf = (void*)new KalmanFilter(A, B, C, Q, R, P);
    filter->is_init = false;
}

void altimetry_filter_update(altimetry_filter_t* filter, FusionAhrs* ahrs) {
    KalmanFilter* Kf = (KalmanFilter*)filter->Kf;
    if (filter->is_init) {
        FusionVector worldacc = FusionVectorMultiplyScalar(FusionAhrsGetEarthAcceleration(ahrs), GRAVITY_G);
        Eigen::VectorXf input(1);
        input.fill(0.0f);
        input[0] = worldacc.axis.z;
        Kf->predict(input);
    }
}
// void altimetry_filter_update(altimetry_filter_t* filter, FusionAhrs* ahrs, float baro_altitude) {
void altimetry_filter_correct(altimetry_filter_t* filter, float baro_altitude) {
    KalmanFilter* Kf = (KalmanFilter*)filter->Kf;
    if (!filter->is_init) {
        // Initialize state with barometric altitude
        Eigen::Vector2f x0 = {
            baro_altitude, 0.0f
        };
        Kf->init(x0);
        filter->is_init = true;
    } else {
        Eigen::VectorXf measurement(1);
        measurement.fill(0.0f);
        measurement[0] = baro_altitude;

        Kf->update(measurement);
    }
}

float altimetry_filter_get_filtered_vspeed(altimetry_filter_t* filter) {
    KalmanFilter* Kf = (KalmanFilter*)filter->Kf;
    return Kf->state()[1];

}
float altimetry_filter_get_filtered_altitude(altimetry_filter_t* filter) {
    KalmanFilter* Kf = (KalmanFilter*)filter->Kf;
    return Kf->state()[0];
}
