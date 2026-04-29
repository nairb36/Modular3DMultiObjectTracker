// Implementation of the Tracker orchestration loop.
// Manages track lifecycle: creation, update, and deletion across frames.


#include "tracker.hpp"

Tracker::Tracker(): next_id_(0),
                    curr_frame_id_(0),
                    time_step_(0)
{

}


void Tracker::tracker_step()
{
    
    // get_detections(); // fills in curr_frame_detections_

    // predict_tracks_state(); // does state prediction based on motion model for each Track in tracks_
    // perform_association();  // Association step: keeps list of matched and unmatched detections
    // update_tracks_state();  // Applies measurement update to matched tracks, leaves unmatched tracks as is

    // Create new tracks from unmatched detections

    // Delete tracks exceeding consecutive miss threshold

    curr_frame_id_++;
    time_step_++;
}


void Tracker::get_detections()
{
    curr_frame_detections_ = detector_->detect(curr_frame_id_);
}