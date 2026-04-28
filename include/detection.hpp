// Represents a single 3D detection at a given timestep.
// Holds position (x, y, z), bounding box dimensions (l, w, h), yaw, and confidence.
// Pure data struct — no behavior.

#include <Eigen/Dense>
#include <string>
#include <vector>

/// @brief A single 3D detection: position, bounding box, and orientation
struct Detection
{
    std::string instance_token;
    std::string category_name;
    float confidence;
    Eigen::Vector3d position; // [x, y, z]
    Eigen::Vector3d bbox_dims; // [l, w, h]
    Eigen::Vector4d rotation_quaternion; 
    double yaw;

};