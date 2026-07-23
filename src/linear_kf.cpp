// Implementation of LinearKF: predict() and update() using standard KF equations.
// Matrix setup (F, H, Q, R, P) happens in the constructor.


#include "linear_kf.hpp"

LinearKF::LinearKF(Eigen::Vector3d position, const MotionModelConfig& config)
{
    // Tunable params from config (diagonals): state = [x, y, z, vx, vy, vz], measurement = [x, y, z]
    Eigen::VectorXd sigma_squared_state =
        Eigen::Map<const Eigen::VectorXd>(config.initial_state_variance.data(), config.initial_state_variance.size());
    Eigen::VectorXd sigma_squared_measurement =
        Eigen::Map<const Eigen::VectorXd>(config.measurement_noise_variance.data(), config.measurement_noise_variance.size());
    Eigen::VectorXd sigma_squared_process =
        Eigen::Map<const Eigen::VectorXd>(config.process_noise_variance.data(), config.process_noise_variance.size());

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


void LinearKF::update(const Eigen::VectorXd& z, const double yaw)
{
    Eigen::VectorXd y = z - H_*x_;
    Eigen::MatrixXd S = H_*P_*H_.transpose() + R_;
    Eigen::MatrixXd K = P_*H_.transpose()*S.inverse();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);

    x_ = x_ + K*y;
    P_ = (I - K*H_)*P_;

    yaw_ = yaw;
}


Eigen::Vector3d LinearKF::get_position() const
{
    return x_.head(3);
}


Eigen::MatrixXd LinearKF::get_covariance() const
{
    return P_;
}

Innovation LinearKF::compute_innovation(const Eigen::VectorXd& z) const
{
    Innovation innov;
    innov.y = z - H_ * x_;
    innov.S = H_ * P_ * H_.transpose() + R_;
    return innov;
}

double LinearKF::get_yaw() const
{
    return yaw_;
}