// Entry point: parses config/arguments, instantiates Detector and Tracker,
// and runs the tracking loop over all frames.

#include "tracker.hpp"
#include "gt_detector.hpp"
#include "linear_kf.hpp"

#include <iostream>
#include <string>
#include <memory>

int main()
{
    // TODO temp. read from a config file later
    std::string scene_path = "../results/gt/scene_0000.json";
    std::string detector_type = "GT";
    std::string motion_model_type = "ConstVelocity";

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

    Tracker mot_tracker(std::move(detector), std::move(motion_model_factory));
    while (true)
    {
        mot_tracker.tracker_step();
    }

}
