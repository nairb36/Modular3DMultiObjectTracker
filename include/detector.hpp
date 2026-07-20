// Abstract interface for producing detections from a data source.
// Each concrete Detector receives a Frame and returns detections.

#pragma once

#include "detection.hpp"
#include "scene.hpp"
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

struct DetectorConfig
{
    std::string type;
    std::vector<std::string> tracked_categories;
    float score_threshold = 0.0f;

    static DetectorConfig from_json(const nlohmann::json& j, const std::string& config_dir)
    {
        DetectorConfig cfg;
        cfg.type = j["type"].get<std::string>();

        std::ifstream params_file(config_dir + "/" + j["params"].get<std::string>());
        nlohmann::json params = nlohmann::json::parse(params_file);
        cfg.tracked_categories = params["tracked_categories"].get<std::vector<std::string>>();
        cfg.score_threshold = params.value("score_threshold", 0.0f); // GT param file has no score_threshold
        return cfg;
    }
};

class Detector
{
    public:
    virtual std::vector<Detection> detect(const Frame&) = 0;
    virtual ~Detector() = default;
};

