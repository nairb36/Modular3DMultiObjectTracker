#pragma once

#include "detector.hpp"
#include <string>

class GTDetector : public Detector
{
    public:
    GTDetector(const DetectorConfig& config, const std::string& detections_file);
    std::vector<Detection> detect(const Frame&) override;

    private:
    std::string detections_file_;
    std::vector<std::string> tracked_categories_;
    bool is_tracked_category(const std::string& category_name);
};
