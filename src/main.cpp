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
    std::string detector_type = config["detector"]["type"];
    std::string motion_model_type = config["motion_model"]["type"];
    std::vector<std::string> cost_types = config["cost_function"]["types"].get<std::vector<std::string>>();
    std::vector<double> cost_weights = config["cost_function"]["weights"].get<std::vector<double>>();
    double distance_gate = config["cost_function"]["distance_gate"];
    int max_consecutive_misses = config["track_management"]["max_consecutive_misses"];

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
