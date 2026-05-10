// Implementation of the Tracker orchestration loop.
// Manages track lifecycle: creation, update, and deletion across frames.


#include "tracker.hpp"

Tracker::Tracker(std::unique_ptr<Detector> detector,
                 std::function<std::unique_ptr<MotionModel>(Eigen::Vector3d)> motion_model_factory,
                 const TrackerConfig& config): next_id_(0),
                                               curr_frame_id_(0),
                                               curr_timestamp_(0),
                                               kMaxConsecutiveMisses(config.max_consecutive_misses),
                                               detector_(std::move(detector)),
                                               motion_model_factory_(std::move(motion_model_factory)),
                                               cost_function_(config.cost_function_config),
                                               associator_(std::make_unique<Associator>(config.associator_config))
{

}


// Runs the full tracking pipeline for one frame
void Tracker::tracker_step()
{
    reset_per_frame_state(); // resets member variables for current timestep
    std::cout<<"Completed reset_per_frame_state"<<std::endl;

    double dt = get_timestamp();
    std::cout<<"Completed get_timestamp"<<std::endl;
    
    get_detections(); // fills in curr_frame_detections_
    std::cout<<"Completed get_detections"<<std::endl;

    predict_tracks_state(dt); // does state prediction based on motion model for each Track in tracks_
    std::cout<<"Completed predict_tracks_state"<<std::endl;
    
    perform_association();  // Association step: keeps list of matched and unmatched detections
    std::cout<<"Completed perform_association"<<std::endl;

    update_tracks_state();  // Applies measurement update to matched tracks, leaves unmatched tracks as is
    std::cout<<"Completed update_tracks_state"<<std::endl;

    create_new_tracks(); // Create new tracks from unmatched detections
    std::cout<<"Completed create_new_tracks"<<std::endl;

    delete_old_tracks(); // Delete tracks exceeding consecutive miss threshold
    std::cout<<"Completed delete_old_tracks"<<std::endl;

    curr_frame_id_++;
}


// Clears per-frame containers before processing a new timestep
void Tracker::reset_per_frame_state()
{
    curr_frame_matched_detections_.clear();
    curr_frame_unmatched_detections_.clear();
    tracks_to_detections_map_.clear();
}


// Fetches detections for the current frame from the detector
void Tracker::get_detections()
{
    curr_frame_detections_ = detector_->detect(curr_frame_id_);
}


// Computes dt between current and previous frame timestamps
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


// Propagates each track's state forward using its motion model
void Tracker::predict_tracks_state(double dt)
{
    for (Track& track: tracks_)
    {
        track.motion_model_->predict(dt); // State prediction based on motion model
    }
}


// Matches tracks to detections via cost matrix + Hungarian, populates matched/unmatched lists
void Tracker::perform_association()
{   
    if (tracks_.empty())
    {
        curr_frame_unmatched_detections_ = curr_frame_detections_;
        return;
    }

    if (curr_frame_detections_.empty())
    {
        return;
    }

    // Builds cost matrix after performing gating
    associator_->build_cost_matrix(tracks_, curr_frame_detections_, cost_function_);
    // Bipartite Matching between tracks and detections
    associator_->perform_bipartite_matching();
    std::vector<int> track_assignment_list = associator_->get_assignment_list(); // vector containing matching detection index for each track. -1 for no match.

    std::unordered_set<int> matched_detections_indices_set;
    std::unordered_set<int> unmatched_detections_indices_set;

    for (int i = 0; i < track_assignment_list.size(); i++)
    {
        if (track_assignment_list[i] >= 0)
        {
            // Detections matched to existing tracks
            matched_detections_indices_set.insert(track_assignment_list[i]);
            curr_frame_matched_detections_.push_back(curr_frame_detections_[track_assignment_list[i]]);
            tracks_to_detections_map_.insert(std::pair<int, int> {i, track_assignment_list[i]});
        }
    }

    for (int i = 0; i < curr_frame_detections_.size(); i++)
    {
        if (matched_detections_indices_set.find(i) == matched_detections_indices_set.end())
        {
            // Unmatched detections
            unmatched_detections_indices_set.insert(i);
            curr_frame_unmatched_detections_.push_back(curr_frame_detections_[i]);
        }
    }
}


// Applies measurement update to matched tracks, increments misses for unmatched
void Tracker::update_tracks_state()
{
    for (int i = 0; i < tracks_.size(); i++)
    {
        if (tracks_to_detections_map_.find(i) != tracks_to_detections_map_.end())
        {
            // Current track is associated with a detection in the current frame
            Detection corresponding_detection = curr_frame_detections_[tracks_to_detections_map_[i]];
            tracks_[i].motion_model_->update(corresponding_detection.position_); // Measurement update for state estimation
            tracks_[i].consecutive_misses_ = 0;
            tracks_[i].hits_++;
            tracks_[i].age_++;
        }
        else
        {
            // Current track is NOT associated with a detection in the current frame
            tracks_[i].consecutive_misses_++;
            tracks_[i].age_++;
        }
    }
}


// Initializes new tracks from detections that were not matched to any existing track
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


// Removes tracks that have exceeded the maximum allowed consecutive misses
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