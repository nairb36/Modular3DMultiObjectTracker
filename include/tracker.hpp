// Main orchestrator that manages all tracks through their lifecycle.
// Runs the per-frame pipeline: predict → associate → update → create → delete.
// Owns the list of active tracks, the Associator, and the Detector.

#include "track.hpp"
#include "detector.hpp"
#include "cost_function.hpp"
#include "associator.hpp"
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <unordered_map>
#include <unordered_set>


class Tracker
{
    private:
    std::vector<Track> tracks_; // Vector storing all the active tracks

    int next_id_;
    int curr_frame_id_;
    double curr_timestamp_;
    // TODO take it in from config later
    int kMaxConsecutiveMisses = 5;

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

    public:
    Tracker(std::unique_ptr<Detector>, std::function<std::unique_ptr<MotionModel>(Eigen::Vector3d)>, const std::vector<std::string>&, const std::vector<double>&);

    void tracker_step();
    void reset_per_frame_state();
    void get_detections();
    double get_timestamp();
    void predict_tracks_state(double);
    void perform_association();
    void update_tracks_state();
    void create_new_tracks();
    void delete_old_tracks();
};

