// Concrete Detector: reads ground-truth bounding boxes from a JSON file.
// Returns detections for a given frame/timestamp from preloaded data.

#pragma once

#include "detector.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>

using json = nlohmann::json;

/// @brief Concrete Detector that loads ground-truth detections from a JSON file
class GTDetector : public Detector
{
    public:
    GTDetector(const std::string& json_path, const DetectorConfig& config);
    std::vector<Detection> detect(int) override;
    double get_timestamp(int) override;

    private:
    json scene_data_;
    std::vector<std::string> tracked_categories_;
    bool is_tracked_category(const std::string& category_name);
};