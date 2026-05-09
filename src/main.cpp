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
    // Configuration
    // TODO temp. read from a config file later
    std::string scene_path = "../results/gt/scene_0000.json";
    std::string detector_type = "GT";
    std::string motion_model_type = "ConstVelocity";
    std::vector<std::string> cost_types = {"distance", "iou"};
    std::vector<double> cost_weights = {0.5, 0.5};

    // Detector and motion model setup
    std::unique_ptr<Detector> detector;
    std::function<std::unique_ptr<MotionModel>(Eigen::Vector3d)> motion_model_factory;

    if (detector_type == "GT")
    {
        detector = std::make_unique<GTDetector>(scene_path);
    }

    if (motion_model_type == "ConstVelocity")
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
    Tracker mot_tracker(std::move(detector), std::move(motion_model_factory), cost_types, cost_weights);
    for (int i = 0; i < num_frames; i++)
    {
        std::cout<<"Frame: "<<i<<std::endl;
        mot_tracker.tracker_step();
        std::cout<<"**************************"<<std::endl;
    }

}
