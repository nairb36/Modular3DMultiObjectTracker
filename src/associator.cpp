// Implementation of track-to-detection association.
// Builds cost matrix using a pluggable cost function, gates, and runs Hungarian.

#include "associator.hpp"

Associator::Associator(const AssociatorConfig& config): kDistanceGate(config.distance_gate)
{

}

void Associator::build_cost_matrix(const std::vector<Track>& tracks, const std::vector<Detection>& detections, CostFunction& cost_function)
{
    int curr_num_tracks = tracks.size();
    int curr_num_detections = detections.size();
    cost_matrix_ = std::vector<std::vector<double>>(curr_num_tracks,
                                                    std::vector<double>(curr_num_detections, std::numeric_limits<double>::max()));

    for (int i = 0; i < curr_num_tracks; i++)
    {
        for (int j = 0; j < curr_num_detections; j++)
        {
            if (apply_gating_rules(tracks[i], detections[j]))
            {
                cost_matrix_[i][j] = cost_function.compute_cost(tracks[i], detections[j]);
            }
        }
    }
}


bool Associator::apply_gating_rules(const Track& track, const Detection& detection)
{
    // TODO write actual implementation of gating logic
    
    // Gate 1: track and detection belong to the same category?
    if (track.category_name_ != detection.category_name_)
    {
        return false;
    }

    return true;
}


void Associator::perform_bipartite_matching()
{
    double assignment_cost = hungarian_algorithm_.Solve(cost_matrix_, assignments_);
}


std::vector<int> Associator::get_assignment_list()
{
    return assignments_;
}