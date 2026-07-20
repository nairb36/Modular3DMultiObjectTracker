// Computes weighted pairwise cost between a track and a detection.
// Supports a configurable mix of cost types (e.g. Euclidean distance, IoU)
// with per-type weights. Tracker calls compute_cost() without knowing the internals.

#pragma once

#include "detection.hpp"
#include "track.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

struct CostFunctionConfig
{
    std::vector<std::string> cost_types;
    std::vector<double> cost_weights;
    double distance_gate = 5.0;

    static CostFunctionConfig from_json(const nlohmann::json& j, const std::string& config_dir)
    {
        CostFunctionConfig cfg;
        // Cost types and weights stay in the MOT config: they describe what runs, not tuning values
        cfg.cost_types = j["types"].get<std::vector<std::string>>();
        cfg.cost_weights = j["weights"].get<std::vector<double>>();

        std::ifstream params_file(config_dir + "/" + j["params"].get<std::string>());
        nlohmann::json params = nlohmann::json::parse(params_file);
        cfg.distance_gate = params["distance_gate"].get<double>();
        return cfg;
    }
};

class CostFunction
{
    private:
    double kDistanceGate;

    std::vector<std::string> cost_types_;
    std::vector<double> cost_weights_;
    double distance_cost(const Track&, const Detection&);
    double iou_cost(const Track&, const Detection&);

    public:
    CostFunction(const CostFunctionConfig&);
    double compute_cost(const Track&, const Detection&);
};