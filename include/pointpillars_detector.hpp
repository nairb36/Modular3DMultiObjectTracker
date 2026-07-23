#pragma once

#include "detector.hpp"
#include <string>

class PointPillarsDetector : public Detector
{
    public:
    PointPillarsDetector(const DetectorConfig& config, const std::string& detections_file);
    std::vector<Detection> detect(const Frame&) override;

    private:
    std::string detections_file_;
    std::vector<std::string> tracked_categories_;
    float score_threshold_;
    bool is_tracked_category(const std::string& category_name);
};
