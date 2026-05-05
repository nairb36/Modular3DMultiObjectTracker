// Main orchestrator that manages all tracks through their lifecycle.
// Runs the per-frame pipeline: predict → associate → update → create → delete.
// Owns the list of active tracks, the Associator, and the Detector.

#include "track.hpp"
#include "detector.hpp"
#include <vector>
#include <memory>


class Tracker
{
    private:
    std::vector<Track> tracks_; // Vector storing all the active tracks

    int next_id_;
    int curr_frame_id_;
    double curr_timestamp_;

    // Detection
    std::unique_ptr<Detector> detector_;
    std::vector<Detection> curr_frame_detections_;
    std::vector<Detection> curr_frame_matched_detections_;
    std::vector<Detection> curr_frame_unmatched_detections_;


    public:
    Tracker();

    void tracker_step();
    void get_detections();
    double get_timestamp();
    void predict_tracks_state();
    void perform_association();
    void update_tracks_state();
};

