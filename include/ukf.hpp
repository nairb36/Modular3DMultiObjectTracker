// Concrete MotionModel: Unscented Kalman Filter with Constant Turning Rate, Velocity (CTRV) model.
// State: [x, y, z, v, yaw, yaw_d]. Measurement: [x, y, z].
// State variable altered from classic CTRV state variable to fit cleanly with the current motion model interface

#pragma once

#include "motion_model.hpp"

class UKF: public MotionModel
{
    private:
    Eigen::VectorXd x_; // State
    Eigen::VectorXd x_aug_; // Augmented State
    Eigen::MatrixXd P_; // State Covariance Matrix
    Eigen::MatrixXd Q_; // System Noise Covariance
    Eigen::MatrixXd P_aug_; // Augmented State Covariance Matrix
    Eigen::MatrixXd H_; // Observation Matrix
    Eigen::MatrixXd R_; // Measurement Noise Covariance Matrix
    double yaw_; // yaw of BBox


    public:
    UKF(Eigen::Vector3d);
    Eigen::Vector3d get_position() const;
    Eigen::MatrixXd get_covariance() const;
    Innovation compute_innovation(const Eigen::VectorXd& z) const;
    double get_yaw() const;
    void predict(double dt);
    void update(const Eigen::VectorXd& measurement, const double yaw);
};
