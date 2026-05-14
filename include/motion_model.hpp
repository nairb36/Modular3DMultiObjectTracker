// Abstract base class for state estimation models.
// Defines the interface: predict(), update(), getState(), getCovariance().
// Concrete implementations: LinearKF (v1), UKF (v2).

#pragma once

#include <Eigen/Dense>
#include <string>
#include <nlohmann/json.hpp>

struct MotionModelConfig
{
    std::string type;

    static MotionModelConfig from_json(const nlohmann::json& j)
    {
        MotionModelConfig cfg;
        cfg.type = j["type"].get<std::string>();
        return cfg;
    }
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