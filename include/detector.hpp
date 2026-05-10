// Abstract interface for producing detections from a data source.
// v1 implementation reads ground-truth boxes from a JSON file.
// Designed so LiDAR/camera/BEV detectors can be swapped in later.

#pragma once

#include "detection.hpp"
#include <string>

struct DetectorConfig
{
    std::string type;
};

/// @brief Abstract base class for detection sources; subclass per sensor/input type
class Detector
{   
    public:
    virtual std::vector<Detection> detect(int) = 0;
    virtual double get_timestamp(int) = 0;
    virtual ~Detector() = default;

};

