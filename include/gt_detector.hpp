// Concrete Detector: reads ground-truth annotations from a Frame.

#pragma once

#include "detector.hpp"
#include <string>

class GTDetector : public Detector
{
    public:
    GTDetector(const DetectorConfig& config);
    std::vector<Detection> detect(const Frame&) override;

    private:
    std::vector<std::string> tracked_categories_;
    bool is_tracked_category(const std::string& category_name);
};