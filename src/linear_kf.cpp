// Implementation of LinearKF: predict() and update() using standard KF equations.
// Matrix setup (F, H, Q, R, P) happens in the constructor.


#include "linear_kf.hpp"

LinearKF::LinearKF(Eigen::Vector3d position)
{
    // Tunable Params
    Eigen::VectorXd sigma_squared_state(6);
    sigma_squared_state<< 100, 100, 100, 20, 20, 5; // sigma squared for x, y, z, vx, vy, vz
    Eigen::VectorXd sigma_squared_measurement(3);
    sigma_squared_measurement<< 4, 4, 4; // sigma squared for x, y, 
    Eigen::VectorXd sigma_squared_process(6);
    sigma_squared_process << 1, 1, 1, 1, 1, 1;

    // Kalman Filter Matrices Initialization
    x_ = Eigen::VectorXd::Zero(6);
    x_.head(3) = position;

    P_ = sigma_squared_state.asDiagonal();

    // Partially filling F matrix. 
    // Only filling in the constant terms, the time dependent terms will be filled in later
    F_ = Eigen::MatrixXd::Identity(6, 6);
    
    Q_ = sigma_squared_process.asDiagonal();

    H_ = Eigen::MatrixXd::Zero(3, 6);
    H_.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity();
        
    R_ = sigma_squared_measurement.asDiagonal();
}