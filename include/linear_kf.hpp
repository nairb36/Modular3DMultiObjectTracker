// Concrete MotionModel: linear Kalman Filter with constant-velocity model.
// State: [x, y, z, vx, vy, vz]. Measurement: [x, y, z].
// Owns and initializes F, H, Q, R, P matrices.

#pragma once

#include "motion_model.hpp"

class LinearKF: public MotionModel
{
    private:
    Eigen::VectorXd x_; // State
    Eigen::MatrixXd P_; // State Covariance Matrix
    Eigen::MatrixXd F_; // State Transition Matrix
    Eigen::MatrixXd Q_; // System Noise Covariance
    Eigen::MatrixXd H_; // Observation Matrix
    Eigen::MatrixXd R_; // Measurement Noise Covariance Matrix
    double yaw_; // Yaw for the bounding box -> computed from state elements without using KF

    
    public:
    LinearKF(Eigen::Vector3d);
    Eigen::Vector3d get_position() const;
    Eigen::MatrixXd get_covariance() const;
    double get_yaw() const;
    void predict(double dt);
    void update(const Eigen::VectorXd& measurement);
    void compute_yaw();
};
