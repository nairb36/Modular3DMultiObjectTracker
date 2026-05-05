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

    // double dt = get_timestamp();

    // predict_tracks_state(); // does state prediction based on motion model for each Track in tracks_
    // perform_association();  // Association step: keeps list of matched and unmatched detections
    // update_tracks_state();  // Applies measurement update to matched tracks, leaves unmatched tracks as is

    // create_new_tracks(); // Create new tracks from unmatched detections

    // delete_old_tracks(); // Delete tracks exceeding consecutive miss threshold

    curr_frame_id_++;
}


void Tracker::get_detections()
{
    curr_frame_detections_ = detector_->detect(curr_frame_id_);
}


double Tracker::get_timestamp()
{
    if (curr_frame_id_ == 0)
    {
        curr_timestamp_ = detector_->get_timestamp(curr_frame_id_);
        return 0;
    }
    else
    {
        double prev_timestamp = curr_timestamp_;
        curr_timestamp_ = detector_->get_timestamp(curr_frame_id_);
        double dt = curr_timestamp_ - prev_timestamp;
        return dt;
    }
}


// void Tracker::predict_tracks_state()
// {

// }


// void Tracker::perform_association()
// {

// }


// void Tracker::update_tracks_state()
// {

// }


void Tracker::create_new_tracks()
{
    for (const auto& unmatched_detection: curr_frame_unmatched_detections_)
    {
        int id = next_id_;
        next_id_++;
        std::string category_name = unmatched_detection.category_name_;
        std::unique_ptr<MotionModel> motion_model = std::make_unique<LinearKF>(unmatched_detection.position_);
        Eigen::Vector3d bbox_dims = unmatched_detection.bbox_dims_;
        double yaw = unmatched_detection.yaw_;

        Track new_track(id, category_name, std::move(motion_model), bbox_dims, yaw);

        tracks_.push_back(std::move(new_track));
    }
}


// void Tracker::delete_old_tracks()
// {

// }