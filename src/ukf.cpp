// Implementation of Unscented Kalman Filter

#include "ukf.hpp"

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
    x_ = Eigen::VectorXd::Zero(6); // [x, y, z, v, yaw, yaw_d]
    x_.head(3) = position;

    x_aug_ = Eigen::VectorXd::Zero(8); // [x, y, z, v, yaw, yaw_d, nu_a, nu_yaw_dd]

    P_ = sigma_squared_state.asDiagonal();

    Q_ = sigma_squared_process.asDiagonal();

    P_aug_ =  Eigen::MatrixXd::Zero(8, 8);
    P_aug_.bottomRightCorner(2, 2) = Q_;

    H_ = Eigen::MatrixXd::Zero(3, 6);
    H_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();

    R_ = sigma_squared_measurement.asDiagonal();
}


void UKF::predict(double dt)
{
    x_aug_.head(6) = x_;
    P_aug_.topLeftCorner(6,6) = P_;

    
}