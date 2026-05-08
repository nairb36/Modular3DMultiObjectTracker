// Implementation of weighted cost computation between a track and a detection.
// Individual cost types (e.g. Euclidean, IoU) are computed and combined using configurable weights.

#include "cost_function.hpp"

CostFunction::CostFunction(const std::vector<std::string>& cost_types, const std::vector<double>& cost_weights): cost_types_(cost_types),
                                                                                                                 cost_weights_(cost_weights)
{

}


double CostFunction::compute_cost(const Detection& detection, const Track& track)
{
    double total_cost = 0;

    for (int i = 0; i < cost_types_.size(); i++)
    {   
        std::string cost_type = cost_types_[i];
        double weight = cost_weights_[i];
        double cost = 0;

        if (cost_type == "distance")
        {
            cost = distance_cost(detection, track);
        }
        else if (cost_type == "iou")
        {
            cost = iou_cost(detection, track);
        }

        total_cost += weight*cost;
    }

    return total_cost;
}