// Abstract base class for state estimation models.
// Defines the interface: predict(), update(), getState(), getCovariance().
// Concrete implementations: LinearKF (v1), UKF (v2).

#pragma once

#include <Eigen/Dense>
#include <string>

struct MotionModelConfig
{
    std::string type;
};

class MotionModel
{
    public:
    virtual Eigen::Vector3d get_position() const = 0;
    virtual Eigen::MatrixXd get_covariance() const = 0;
    virtual void predict(double dt) = 0;
    virtual void update(const Eigen::VectorXd& measurement) = 0;
    virtual ~MotionModel() = default;
};