// Abstract base class for state estimation models.
// Defines the interface: predict(), update(), getState(), getCovariance().
// Concrete implementations: LinearKF (v1), UKF (v2).

#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>

struct MotionModelConfig
{
    std::string type;
    // Diagonals of P, Q, R. Lengths depend on the motion model's state layout.
    std::vector<double> initial_state_variance;
    std::vector<double> process_noise_variance;
    std::vector<double> measurement_noise_variance;

    static MotionModelConfig from_json(const nlohmann::json& j, const std::string& config_dir)
    {
        MotionModelConfig cfg;
        cfg.type = j["type"].get<std::string>();

        std::ifstream params_file(config_dir + "/" + j["params"].get<std::string>());
        nlohmann::json params = nlohmann::json::parse(params_file);
        cfg.initial_state_variance = params["initial_state_variance"].get<std::vector<double>>();
        cfg.process_noise_variance = params["process_noise_variance"].get<std::vector<double>>();
        cfg.measurement_noise_variance = params["measurement_noise_variance"].get<std::vector<double>>();
        return cfg;
    }
};

struct Innovation
{
    Eigen::VectorXd y;  // z - H*x
    Eigen::MatrixXd S;  // H*P*H' + R
};

class MotionModel
{
    public:
    virtual Eigen::Vector3d get_position() const = 0;
    virtual Eigen::MatrixXd get_covariance() const = 0;
    virtual double get_yaw() const = 0;
    virtual Innovation compute_innovation(const Eigen::VectorXd& z) const = 0;
    virtual void predict(double dt) = 0;
    virtual void update(const Eigen::VectorXd& measurement, const double yaw) = 0;
    virtual ~MotionModel() = default;
};