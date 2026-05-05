// Implementation of GroundTruthDetector: parses JSON, indexes by frame, returns detections.

#include "gt_detector.hpp"

#include <iostream>

GTDetector::GTDetector(const std::string& json_path)
{
    std::ifstream file(json_path);
    scene_data_ = json::parse(file);
}

std::vector<Detection> GTDetector::detect(int frame_id)
{
    std::vector<Detection> detections;
    json curr_frame_ = scene_data_[frame_id];
    json curr_detections = curr_frame_["detections"];

    for (int i = 0; i < curr_detections.size(); i++)
    {
        Detection detection;
        detection.instance_token_ = curr_detections[i]["instance_token"];
        detection.category_name_ = curr_detections[i]["category_name"];
        detection.confidence_ = 1.0; // Ground truth
        auto t = curr_detections[i]["translation"];
        detection.position_ = Eigen::Vector3d(t[0], t[1], t[2]);
        auto s = curr_detections[i]["size"];
        detection.bbox_dims_ = Eigen::Vector3d(s[0], s[1], s[2]);
        auto r = curr_detections[i]["rotation"];
        detection.rotation_quaternion_ = Eigen::Vector4d(r[0], r[1], r[2], r[3]);
        detection.yaw_ = curr_detections[i]["yaw"];

        detections.push_back(detection);
    }

    return detections;
}

double GTDetector::get_timestamp(int frame_id)
{
    json curr_frame_ = scene_data_[frame_id];
    double curr_timestamp = curr_frame_["timestamp"]/1e6;
    return curr_timestamp;
}


// int main()
// {
//     std::unique_ptr<Detector> d = std::make_unique<GTDetector>("../results/gt/scene_0000.json");
//     std::vector<Detection> detections = d->detect(0);
//     std::cout<<"**************";

//     for (int i = 0; i < detections.size(); i++)
//     {
//         std::cout<<(i+1)<<". "<<detections[i].category_name_<<std::endl;
//     }
// }