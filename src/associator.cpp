// Implementation of track-to-detection association.
// Builds cost matrix using a pluggable cost function, gates, and runs Hungarian.

#include "associator.hpp"

void Associator::build_cost_matrix(const std::vector<Track>& tracks, const std::vector<Detection>& detections)
{
    int curr_num_tracks = tracks.size();
    int curr_num_detections = detections.size();
    cost_matrix_ = std::vector<std::vector<double>>(curr_num_tracks, 
                                                    std::vector<double>(curr_num_detections, std::numeric_limits<double>::infinity()));

    for (int i = 0; i < curr_num_tracks; i++)
    {
        for (int j = 0; j < curr_num_detections; j++)
        {
            if (apply_gating_rules(tracks[i], detections[j]))
            {
                cost_matrix_[i][j] = cost_function_.compute_cost(tracks[i], detections[j]);
            }
        }
    }
}

bool Associator::apply_gating_rules()
{
    // TODO write actual implementation of gating logic
    return true;
}