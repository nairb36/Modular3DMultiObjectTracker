// Main orchestrator that manages all tracks through their lifecycle.
// Runs the per-frame pipeline: predict → associate → update → create → delete.
// Owns the list of active tracks, the Associator, and the Detector.

#pragma once

#include "track.hpp"
#include "scene.hpp"
#include "detector.hpp"
#include "motion_model.hpp"
#include "cost_function.hpp"
#include "associator.hpp"
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

struct TrackerConfig
{
    DetectorConfig detector_config;
    MotionModelConfig motion_model_config;
    CostFunctionConfig cost_function_config;
    AssociatorConfig associator_config;
    int max_consecutive_misses = 5;

    // config_dir is the directory of the top-level MOT config file;
    // all "params" paths inside the config resolve relative to it.
    static TrackerConfig from_json(const nlohmann::json& j, const std::string& config_dir)
    {
        TrackerConfig cfg;
        cfg.detector_config = DetectorConfig::from_json(j["detector"], config_dir);
        cfg.motion_model_config = MotionModelConfig::from_json(j["motion_model"], config_dir);
        cfg.cost_function_config = CostFunctionConfig::from_json(j["cost_function"], config_dir);
        cfg.associator_config = AssociatorConfig::from_json(j["associator"], config_dir);

        std::ifstream params_file(config_dir + "/" + j["track_management"]["params"].get<std::string>());
        nlohmann::json params = nlohmann::json::parse(params_file);
        cfg.max_consecutive_misses = params["max_consecutive_misses"].get<int>();
        return cfg;
    }
};

class Tracker
{
    private:
    std::vector<Track> tracks_;
    const Scene& scene_;

    int next_id_;
    int curr_frame_id_;
    int kMaxConsecutiveMisses;

    // Detection
    std::unique_ptr<Detector> detector_;
    std::vector<Detection> curr_frame_detections_;
    std::vector<Detection> curr_frame_matched_detections_;
    std::vector<Detection> curr_frame_unmatched_detections_;

    // Motion Model
    std::function<std::unique_ptr<MotionModel>(Eigen::Vector3d)> motion_model_factory_;

    // Cost Function
    CostFunction cost_function_;

    // Associator
    std::unique_ptr<Associator> associator_;
    std::unordered_map<int, int> tracks_to_detections_map_;

    // Results
    nlohmann::json results_log_;

    // Member Functions
    void reset_per_frame_state();
    void get_detections();
    double get_timestamp();
    void predict_tracks_state(double);
    void perform_association();
    void update_tracks_state();
    void create_new_tracks();
    void delete_old_tracks();
    void log_tracker_results();

    public:
    Tracker(const Scene&, std::unique_ptr<Detector>, std::function<std::unique_ptr<MotionModel>(Eigen::Vector3d)>, const TrackerConfig&);
    void tracker_step();
    std::string save_results(const std::string& output_dir, const std::string& scene_name);

};

