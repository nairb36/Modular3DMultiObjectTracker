// Computes weighted pairwise cost between a track and a detection.
// Supports a configurable mix of cost types (e.g. Euclidean distance, IoU)
// with per-type weights. Tracker calls compute_cost() without knowing the internals.

#include <vector>
#include <string>

class CostFunction
{
    private:
    std::vector<std::string> cost_types_;
    std::vector<double> cost_weights_;

    public:
    CostFunction(std::vector<std::string>, std::vector<double>);
    double distance_cost();
    double iou_cost();
};