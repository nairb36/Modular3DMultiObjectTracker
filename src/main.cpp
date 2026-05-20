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
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

int main()
{
    std::string config_path = "../configs/MOT_v1.json";
    std::ifstream config_file(config_path);
    nlohmann::json config = nlohmann::json::parse(config_file);

    std::string gt_dir = config["data"]["gt_dir"];
    TrackerConfig tracker_config = TrackerConfig::from_json(config);

    // Create timestamped run folder
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ts;
    ts << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    std::string run_dir = "../results/tracking/" + ts.str();

    // Collect all scene JSON files
    std::vector<std::string> scene_files;
    for (const auto& entry : fs::directory_iterator(gt_dir))
    {
        std::string fname = entry.path().filename().string();
        if (entry.path().extension() == ".json" &&
            fname.rfind("scene_", 0) == 0)
        {
            scene_files.push_back(entry.path().string());
        }
    }
    std::sort(scene_files.begin(), scene_files.end());

    std::cout << "Found " << scene_files.size() << " scenes in " << gt_dir << std::endl;

    for (const auto& scene_path : scene_files)
    {
        std::string scene_name = fs::path(scene_path).stem().string();
        std::cout << "\n=== " << scene_name << " ===" << std::endl;

        // Create detector
        std::unique_ptr<Detector> detector;
        if (tracker_config.detector_config.type == "GT")
        {
            detector = std::make_unique<GTDetector>(scene_path, tracker_config.detector_config);
        }

        // Create motion model factory
        std::function<std::unique_ptr<MotionModel>(Eigen::Vector3d)> motion_model_factory;
        if (tracker_config.motion_model_config.type == "ConstVelocity")
        {
            motion_model_factory = [](Eigen::Vector3d position)
            {
                return std::make_unique<LinearKF>(position);
            };
        }

        // Get number of frames
        std::ifstream scene_file(scene_path);
        nlohmann::json scene_json = nlohmann::json::parse(scene_file);
        int num_frames = scene_json.size();

        // Run tracker
        Tracker mot_tracker(std::move(detector), std::move(motion_model_factory), tracker_config);
        for (int i = 0; i < num_frames; i++)
        {
            std::cout << "Frame: " << i << std::endl;
            mot_tracker.tracker_step();
            std::cout << "**************************" << std::endl;
        }

        std::string results_path = mot_tracker.save_results(run_dir, scene_name);
        std::cout << "Wrote results to " << results_path << std::endl;
    }

    std::cout << "\nAll scenes saved to " << run_dir << std::endl;
}
