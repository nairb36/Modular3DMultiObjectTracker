// Matches predicted tracks to current detections each timestep.
// Builds a cost matrix, applies gating, and solves with Hungarian algorithm.
// Returns matched pairs, unmatched tracks, and unmatched detections.

#include "track.hpp"
#include "detection.hpp"
#include <limits>

class Associator
{
    public:
    Associator() = default;
    void build_cost_matrix(const std::vector<Track>&, const std::vector<Detection>&);


    private:
    std::vector<std::vector<double>> cost_matrix_;
    bool apply_gating_rules(const Track&, const Detection&);

};