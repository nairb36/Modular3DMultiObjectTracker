// Implementation of the Tracker orchestration loop.
// Manages track lifecycle: creation, update, and deletion across frames.


#include "tracker.hpp"

Tracker::Tracker(std::unique_ptr<Detector> detector, 
                 std::function<std::unique_ptr<MotionModel>(Eigen::Vector3d)> motion_model_factory, 
                 const std::vector<std::string>& cost_types, 
                 const std::vector<double>& cost_weights): next_id_(0),
                                                           curr_frame_id_(0),
                                                           curr_timestamp_(0),
                                                           detector_(std::move(detector)),
                                                           motion_model_factory_(std::move(motion_model_factory)),
                                                           cost_function_(CostFunction(cost_types, cost_weights))
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


void Tracker::predict_tracks_state(double dt)
{
    for (Track& track: tracks_)
    {
        track.motion_model_->predict(dt);
    }
}


void Tracker::perform_association()
{
    // TODO Actual implementation
    
    // // temp implementation
    // curr_frame_unmatched_detections_ = curr_frame_detections_;

    int curr_num_tracks = tracks_.size();
    int curr_num_detections = curr_frame_detections_.size();
    std::vector<std::vector<double>> cost_matrix(curr_num_tracks, std::vector<double>(curr_num_detections, std::numeric_limits<double>::infinity()));

    for (int i = 0; i < curr_num_tracks; i++)
    {
        for (int j = 0; j < curr_num_detections; j++)
        {
            //TODO Apply gating
                // check_all_gating() 
                // Dont compute pairwise costs if certain criteria is not met
                // eg: dont compute pairwise cost if:
                                                    // class labels are different
                                                    // boxes are too far apart ..
            
            cost_matrix[i][j] = cost_function_.compute_cost(tracks_[i], curr_frame_detections_[j]);
        }
    }

}


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
        std::unique_ptr<MotionModel> motion_model = motion_model_factory_(unmatched_detection.position_);
        Eigen::Vector3d bbox_dims = unmatched_detection.bbox_dims_;
        double yaw = unmatched_detection.yaw_;

        Track new_track(id, category_name, std::move(motion_model), bbox_dims, yaw);

        tracks_.push_back(std::move(new_track));
    }
}


void Tracker::delete_old_tracks()
{
    for (auto itr = tracks_.begin(); itr != tracks_.end(); )
    {
        if (itr->consecutive_misses_ > kMaxConsecutiveMisses)
        {
            itr = tracks_.erase(itr); // Returns the next valid iterator
        }
        else
        {
            itr++; // Only increment if no deletion occurred
        }
    }
}