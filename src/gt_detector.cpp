// Implementation of GTDetector: reads ground-truth annotations from a Frame.

#include "gt_detector.hpp"

GTDetector::GTDetector(const DetectorConfig& config)
    : tracked_categories_(config.tracked_categories)
{
}

std::vector<Detection> GTDetector::detect(const Frame& frame)
{
    std::vector<Detection> detections;

    for (const auto& ann : frame.annotations)
    {
        if (!is_tracked_category(ann.category_name))
            continue;

        Detection detection;
        detection.instance_token_ = ann.instance_token;
        detection.category_name_ = ann.category_name;
        detection.confidence_ = 1.0;
        detection.position_ = ann.translation;
        detection.bbox_dims_ = ann.size;
        detection.rotation_quaternion_ = ann.rotation;
        detection.yaw_ = ann.yaw;
        detections.push_back(detection);
    }

    return detections;
}

bool GTDetector::is_tracked_category(const std::string& category_name)
{
    for (const auto& prefix : tracked_categories_)
    {
        if (category_name.rfind(prefix, 0) == 0)
            return true;
    }
    return false;
}

