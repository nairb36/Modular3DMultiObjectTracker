#pragma once

#include "detector.hpp"
#include <string>

constexpr float PP_SCORE_THRESHOLD = 0.5f;

class PointPillarsDetector : public Detector
{
    public:
    PointPillarsDetector(const DetectorConfig& config, const std::string& detections_file);
    std::vector<Detection> detect(const Frame&) override;

    private:
    std::string detections_file_;
    std::vector<std::string> tracked_categories_;
    bool is_tracked_category(const std::string& category_name);
};
