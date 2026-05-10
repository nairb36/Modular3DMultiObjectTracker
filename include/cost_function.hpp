// Computes weighted pairwise cost between a track and a detection.
// Supports a configurable mix of cost types (e.g. Euclidean distance, IoU)
// with per-type weights. Tracker calls compute_cost() without knowing the internals.

#pragma once

#include "detection.hpp"
#include "track.hpp"
#include <vector>
#include <string>
#include <Eigen/Dense>

struct CostFunctionConfig
{
    std::vector<std::string> cost_types;
    std::vector<double> cost_weights;
    double distance_gate = 5.0;
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