// Entry point: parses config/arguments, instantiates Detector and Tracker,
// and runs the tracking loop over all frames.

#include "tracker.hpp"
#include "gt_detector.hpp"
#include "linear_kf.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <nlohmann/json.hpp>

int main()
{
    // Load configuration
    std::string config_path = "../configs/MOT_v1.json";
    std::ifstream config_file(config_path);
    nlohmann::json config = nlohmann::json::parse(config_file);

    std::string scene_path = config["data"]["scene_path"];

    TrackerConfig tracker_config = TrackerConfig::from_json(config);

    // Detector and motion model setup
    std::unique_ptr<Detector> detector;
    std::function<std::unique_ptr<MotionModel>(Eigen::Vector3d)> motion_model_factory;

    if (tracker_config.detector_config.type == "GT")
    {
        detector = std::make_unique<GTDetector>(scene_path, tracker_config.detector_config);
    }

    if (tracker_config.motion_model_config.type == "ConstVelocity")
    {
        motion_model_factory = [](Eigen::Vector3d position)
        {
            return std::make_unique<LinearKF>(position);
        };
    }

    // Parse scene to get total number of frames
    std::ifstream scene_file(scene_path);
    nlohmann::json scene_json = nlohmann::json::parse(scene_file);
    int num_frames = scene_json.size();

    // Run tracker over all frames
    Tracker mot_tracker(std::move(detector), std::move(motion_model_factory), tracker_config);
    for (int i = 0; i < num_frames; i++)
    {
        std::cout<<"Frame: "<<i<<std::endl;
        mot_tracker.tracker_step();
        std::cout<<"**************************"<<std::endl;
    }

    std::string results_path = mot_tracker.save_results("../results/tracking");
    std::cout << "Wrote results to " << results_path << std::endl;
}
