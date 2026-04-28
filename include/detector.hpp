// Abstract interface for producing detections from a data source.
// v1 implementation reads ground-truth boxes from a JSON file.
// Designed so LiDAR/camera/BEV detectors can be swapped in later.

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <memory>


/// @brief A single 3D detection: position, bounding box, and orientation
struct Detection
{
    std::string instance_token;
    std::string category_name;
    Eigen::Vector3d position; // [x, y, z]
    Eigen::Vector3d bbox_dims; // [l, w, h]
    Eigen::Vector4d rotation_quaternion; 
    double yaw;

};

/// @brief Abstract base class for detection sources; subclass per sensor/input type
class Detector
{   
    public:
    virtual std::vector<Detection> detect(int) = 0;
    virtual ~Detector() = default;

};

