// Computes weighted pairwise cost between a track and a detection.
// Supports a configurable mix of cost types (e.g. Euclidean distance, IoU)
// with per-type weights. Tracker calls compute_cost() without knowing the internals.

#include "detection.hpp"
#include "track.hpp"
#include <vector>
#include <string>

class CostFunction
{
    private:
    std::vector<std::string> cost_types_;
    std::vector<double> cost_weights_;
    double distance_cost(const Detection&, const Track&);
    double iou_cost(const Detection&, const Track&);

    public:
    CostFunction(const std::vector<std::string>&, const std::vector<double>&);
    double compute_cost(const Detection&, const Track&);
};