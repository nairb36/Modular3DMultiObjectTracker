// Implementation of Track: constructor and lifecycle management.
// predict() delegates to MotionModel, update() corrects state and refreshes bbox/yaw.

#include "track.hpp"

Track::Track(int id, std::string category_name, std::unique_ptr<MotionModel> motion_model, Eigen::Vector3d bbox_dims, double yaw)
{
    id_ = id;
    category_name_ = category_name;
    motion_model_ = std::move(motion_model);

    consecutive_misses_ = 0;
    hits_ = 1;
    age_ = 1;

    // Optional metadata
    bbox_dims_ = bbox_dims;
    yaw_ = yaw;
}