// Implementation of weighted cost computation between a track and a detection.
// Individual cost types (e.g. Euclidean, IoU) are computed and combined using configurable weights.

#include "cost_function.hpp"

CostFunction::CostFunction(const std::vector<std::string>& cost_types, const std::vector<double>& cost_weights): cost_types_(cost_types),
                                                                                                                 cost_weights_(cost_weights)
{

}


double CostFunction::compute_cost(const Track& track, const Detection& detection)
{
    double total_cost = 0;

    for (int i = 0; i < cost_types_.size(); i++)
    {   
        std::string cost_type = cost_types_[i];
        double weight = cost_weights_[i];
        double cost = 0;

        if (cost_type == "distance")
        {
            cost = distance_cost(track, detection);
        }
        else if (cost_type == "iou")
        {
            cost = iou_cost(track, detection);
        }

        total_cost += weight*cost;
    }

    return total_cost;
}


double CostFunction::distance_cost(const Track& track, const Detection& detection)
{
    Eigen::Vector3d detection_position = detection.position_;
    Eigen::Vector3d track_position = track.motion_model_->get_position();
    Eigen::Vector3d distance_vector = track_position - detection_position;
    double distance = distance_vector.norm();
    return distance/kDistanceGate; // Normalizing for distance cost
}


// Axis-aligned 3D IoU — does not account for yaw rotation
double CostFunction::iou_cost(const Track& track, const Detection& detection)
{
    Eigen::Vector3d det_pos = detection.position_;
    Eigen::Vector3d det_dims = detection.bbox_dims_;
    Eigen::Vector3d trk_pos = track.motion_model_->get_position();
    Eigen::Vector3d trk_dims = track.bbox_dims_;

    double overlap_x = std::max(0.0, std::min(det_pos(0) + det_dims(0)/2, trk_pos(0) + trk_dims(0)/2)
                                    - std::max(det_pos(0) - det_dims(0)/2, trk_pos(0) - trk_dims(0)/2));
    double overlap_y = std::max(0.0, std::min(det_pos(1) + det_dims(1)/2, trk_pos(1) + trk_dims(1)/2)
                                    - std::max(det_pos(1) - det_dims(1)/2, trk_pos(1) - trk_dims(1)/2));
    double overlap_z = std::max(0.0, std::min(det_pos(2) + det_dims(2)/2, trk_pos(2) + trk_dims(2)/2)
                                    - std::max(det_pos(2) - det_dims(2)/2, trk_pos(2) - trk_dims(2)/2));

    double intersection_vol = overlap_x * overlap_y * overlap_z;
    double vol_det = det_dims(0) * det_dims(1) * det_dims(2);
    double vol_trk = trk_dims(0) * trk_dims(1) * trk_dims(2);
    double union_vol = vol_det + vol_trk - intersection_vol;

    double iou = (union_vol > 0) ? intersection_vol / union_vol : 0;
    return 1.0 - iou; // Higher IoU should lead to lower cost
}