// Implementation of Unscented Kalman Filter

#include "ukf.hpp"
#include <cmath>

UKF::UKF(Eigen::Vector3d position)
{
    // Tunable Params
    Eigen::VectorXd sigma_squared_state(6);
    sigma_squared_state<< 100, 100, 100, 20, 3, 1; // sigma squared for x, y, z, v, yaw, yaw_d
    Eigen::VectorXd sigma_squared_measurement(3);
    sigma_squared_measurement<< 0.1, 0.1, 0.1; // sigma squared for x, y, z
    Eigen::VectorXd sigma_squared_process(2);
    sigma_squared_process << 1, 1;

    // UKF Matrices Initialization
    n_ = 6;
    n_aug_ = 8;
    x_ = Eigen::VectorXd::Zero(n_); // [x, y, z, v, yaw, yaw_d]
    x_.head(3) = position;

    x_aug_ = Eigen::VectorXd::Zero(n_aug_); // [x, y, z, v, yaw, yaw_d, nu_a, nu_yaw_dd]

    P_ = sigma_squared_state.asDiagonal();

    Q_ = sigma_squared_process.asDiagonal();

    P_aug_ =  Eigen::MatrixXd::Zero(n_aug_, n_aug_);
    P_aug_.bottomRightCorner(2, 2) = Q_;

    H_ = Eigen::MatrixXd::Zero(3, n_);
    H_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();

    R_ = sigma_squared_measurement.asDiagonal();
}


void UKF::predict(double dt)
{
    x_aug_.head(n_) = x_;
    P_aug_.topLeftCorner(n_,n_) = P_;

    // Generate Sigma Points
    Eigen::MatrixXd X_aug = generate_sigma_points();

    // Propogate Sigma Points

    // Compute mean and covariance of predicted Sigma Points
}

Eigen::MatrixXd UKF::generate_sigma_points() const
{
    double lambda = 3.0 - n_aug_;

    // Matrix square root of P_aug via Cholesky decomposition: P_aug = L*L'
    Eigen::MatrixXd L = P_aug_.llt().matrixL();
    Eigen::MatrixXd scaled_L = std::sqrt(lambda + n_aug_) * L;

    Eigen::MatrixXd X_aug(n_aug_, 2*n_aug_ + 1);
    X_aug.col(0) = x_aug_;
    X_aug.block(0, 1, n_aug_, n_aug_) = scaled_L.colwise() + x_aug_;
    X_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) = (-scaled_L).colwise() + x_aug_;
    return X_aug;
}
