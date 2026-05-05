// Implementation of LinearKF: predict() and update() using standard KF equations.
// Matrix setup (F, H, Q, R, P) happens in the constructor.


#include "linear_kf.hpp"

LinearKF::LinearKF(Eigen::Vector3d position)
{
    // Tunable Params
    Eigen::VectorXd sigma_squared_state(6);
    sigma_squared_state<< 100, 100, 100, 20, 20, 5; // sigma squared for x, y, z, vx, vy, vz
    Eigen::VectorXd sigma_squared_measurement(3);
    sigma_squared_measurement<< 4, 4, 4; // sigma squared for x, y, z
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


void LinearKF::predict(double dt)
{
    // Updating relevant parts of the F matrix that depend on dt
    F_.topRightCorner(3,3) = dt*Eigen::Matrix3d::Identity(3,3);

    x_ = F_*x_;
    P_ = F_*P_*F_.transpose() + Q_;
}


void LinearKF::update(Eigen::VectorXd z)
{
    Eigen::VectorXd y = z - H_*x_;
    Eigen::MatrixXd S = H_*P_*H_.transpose() + R_;
    Eigen::MatrixXd K = P_*H_.transpose()*S.inverse();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);

    x_ = x_ + K*y;
    P_ = (I - K*H_)*P_;
}


Eigen::Vector3d LinearKF::get_position()
{
    return x_.head(3);
}


Eigen::MatrixXd LinearKF::get_covariance()
{
    return P_;
}