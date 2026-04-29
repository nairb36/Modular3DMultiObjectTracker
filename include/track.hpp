// Represents a single tracked object through time.
// Owns a KalmanFilter for state estimation plus passive metadata (bbox dims, yaw).
// Tracks lifecycle info: age, hits, consecutive misses, track ID.

#include <string>
#include <memory>
#include <Eigen/Dense>

struct Track
{
    int id_;
    std::string category_name_;
    std::unique_ptr<MotionModel> motion_model_;
    int consecutive_misses_;
    int hits_;
    int age_;

    // Optional Metadata
    Eigen::Vector3d bbox_dims_;
    double yaw_;
};