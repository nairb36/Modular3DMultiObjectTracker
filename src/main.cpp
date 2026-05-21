// Entry point: parses config, loads Scenes, instantiates Detector and Tracker,
// and runs the tracking loop over all frames.

#include "tracker.hpp"
#include "gt_detector.hpp"
#include "linear_kf.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

int main()
{
    std::string config_path = "../configs/MOT_v1.json";
    std::ifstream config_file(config_path);
    nlohmann::json config = nlohmann::json::parse(config_file);

    std::string scene_dir = config["data"]["scene_dir"];
    TrackerConfig tracker_config = TrackerConfig::from_json(config);

    // Create timestamped run folder
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ts;
    ts << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    std::string run_dir = "../results/tracking/" + ts.str();

    // Collect all scene JSON files
    std::vector<std::string> scene_files;
    for (const auto& entry : fs::directory_iterator(scene_dir))
    {
        std::string fname = entry.path().filename().string();
        if (entry.path().extension() == ".json" &&
            fname.rfind("scene_", 0) == 0)
        {
            scene_files.push_back(entry.path().string());
        }
    }
    std::sort(scene_files.begin(), scene_files.end());

    std::cout << "Found " << scene_files.size() << " scenes in " << scene_dir << std::endl;

    for (const auto& scene_path : scene_files)
    {
        Scene scene = Scene::from_json(scene_path);
        std::cout << "\n=== " << scene.scene_name << " (" << scene.num_frames() << " frames) ===" << std::endl;

        // Create detector
        std::unique_ptr<Detector> detector;
        if (tracker_config.detector_config.type == "GT")
        {
            detector = std::make_unique<GTDetector>(tracker_config.detector_config);
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

        // Run tracker
        std::string scene_stem = fs::path(scene_path).stem().string();
        Tracker mot_tracker(scene, std::move(detector), std::move(motion_model_factory), tracker_config);
        for (int i = 0; i < scene.num_frames(); i++)
        {
            std::cout << "Frame: " << i << std::endl;
            mot_tracker.tracker_step();
            std::cout << "**************************" << std::endl;
        }

        std::string results_path = mot_tracker.save_results(run_dir, scene_stem);
        std::cout << "Wrote results to " << results_path << std::endl;
    }

    std::cout << "\nAll scenes saved to " << run_dir << std::endl;
}
