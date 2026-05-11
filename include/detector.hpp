// Abstract interface for producing detections from a data source.
// v1 implementation reads ground-truth boxes from a JSON file.
// Designed so LiDAR/camera/BEV detectors can be swapped in later.

#pragma once

#include "detection.hpp"
#include <string>
#include <nlohmann/json.hpp>

struct DetectorConfig
{
    std::string type;

    static DetectorConfig from_json(const nlohmann::json& j)
    {
        DetectorConfig cfg;
        cfg.type = j["type"].get<std::string>();
        return cfg;
    }
};

/// @brief Abstract base class for detection sources; subclass per sensor/input type
class Detector
{   
    public:
    virtual std::vector<Detection> detect(int) = 0;
    virtual double get_timestamp(int) = 0;
    virtual ~Detector() = default;

};

