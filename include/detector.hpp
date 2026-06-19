// Abstract interface for producing detections from a data source.
// Each concrete Detector receives a Frame and returns detections.

#pragma once

#include "detection.hpp"
#include "scene.hpp"
#include <string>
#include <nlohmann/json.hpp>

struct DetectorConfig
{
    std::string data_root;
    std::string type;
    std::vector<std::string> tracked_categories;

    static DetectorConfig from_json(const nlohmann::json& j)
    {
        DetectorConfig cfg;
        if (j.contains("data_root"))
            cfg.data_root = j["data_root"].get<std::string>();
        cfg.type = j["type"].get<std::string>();
        cfg.tracked_categories = j["tracked_categories"].get<std::vector<std::string>>();
        return cfg;
    }
};

class Detector
{
    public:
    virtual std::vector<Detection> detect(const Frame&) = 0;
    virtual ~Detector() = default;
};

