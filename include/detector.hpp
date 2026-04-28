// Abstract interface for producing detections from a data source.
// v1 implementation reads ground-truth boxes from a JSON file.
// Designed so LiDAR/camera/BEV detectors can be swapped in later.

#include "detection.hpp"


/// @brief Abstract base class for detection sources; subclass per sensor/input type
class Detector
{   
    public:
    virtual std::vector<Detection> detect(int) = 0;
    virtual ~Detector() = default;

};

