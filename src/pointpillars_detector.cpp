#include "pointpillars_detector.hpp"
#include <fstream>

PointPillarsDetector::PointPillarsDetector(const DetectorConfig& config, const std::string& detections_file)
    : detections_file_(detections_file),
      tracked_categories_(config.tracked_categories)
{
}

std::vector<Detection> PointPillarsDetector::detect(const Frame& frame)
{
    std::vector<Detection> detections;

    std::ifstream f(detections_file_);
    nlohmann::json j = nlohmann::json::parse(f);

    for (const auto& frame_json : j)
    {
        if (frame_json["sample_token"] != frame.sample_token)
            continue;

        for (const auto& dj : frame_json["detections"])
        {
            float score = dj["score"].get<float>();
            if (score < PP_SCORE_THRESHOLD)
                continue;

            std::string category = dj["category_name"].get<std::string>();
            if (!is_tracked_category(category))
                continue;

            auto t = dj["translation"].get<std::vector<double>>();
            auto s = dj["size"].get<std::vector<double>>();
            auto r = dj["rotation"].get<std::vector<double>>();

            Detection det;
            det.category_name_ = category;
            det.confidence_ = score;
            det.position_ = Eigen::Vector3d(t[0], t[1], t[2]);
            det.bbox_dims_ = Eigen::Vector3d(s[0], s[1], s[2]);
            det.rotation_quaternion_ = Eigen::Vector4d(r[0], r[1], r[2], r[3]);
            det.yaw_ = dj["yaw"].get<double>();
            detections.push_back(std::move(det));
        }

        break;
    }

    return detections;
}

bool PointPillarsDetector::is_tracked_category(const std::string& category_name)
{
    for (const auto& prefix : tracked_categories_)
    {
        if (category_name.rfind(prefix, 0) == 0)
            return true;
    }
    return false;
}
