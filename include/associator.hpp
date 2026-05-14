// Matches predicted tracks to current detections each timestep.
// Builds a cost matrix, applies gating, and solves with Hungarian algorithm.
// Returns matched pairs, unmatched tracks, and unmatched detections.

#pragma once

#include "track.hpp"
#include "detection.hpp"
#include "Hungarian.h"
#include "cost_function.hpp"
#include <limits>
#include <nlohmann/json.hpp>

struct AssociatorConfig
{
    double distance_gate = 5.0;

    static AssociatorConfig from_json(const nlohmann::json& j)
    {
        AssociatorConfig cfg;
        cfg.distance_gate = j["distance_gate"].get<double>();
        return cfg;
    }
};

class Associator
{
    public:
    Associator(const AssociatorConfig& config);
    void build_cost_matrix(const std::vector<Track>&, const std::vector<Detection>&, CostFunction&);
    void perform_bipartite_matching();
    std::vector<int> get_assignment_list();


    private:
    double kDistanceGate;
    HungarianAlgorithm hungarian_algorithm_;
    std::vector<std::vector<double>> cost_matrix_;
    std::vector<int> assignments_;
    bool apply_gating_rules(const Track&, const Detection&);

};