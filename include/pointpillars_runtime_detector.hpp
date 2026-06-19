#pragma once

#include "detector.hpp"
#include "scene.hpp"
#include <onnxruntime_cxx_api.h>

class PointPillarsRuntimeDetector : public Detector
{
    public:
    std::vector<Detection> detect(const Frame&) override;
    PointPillarsRuntimeDetector(const DetectorConfig& config, const Calibration& lidar_calib);

    private:
    std::string data_root_;
    std::vector<std::string> tracked_categories_;
    Calibration lidar_calib_;

    Ort::Env env_;
    Ort::Session pfe_session_;
    Ort::Session backbone_session_;

    std::vector<float> point_cloud_;
    std::vector<float> voxels_;
    std::vector<int64_t> voxel_num_points_;
    std::vector<int> voxel_coords_;
    std::vector<float> pillar_features_;
    std::vector<float> spatial_features_;
    std::vector<float> cls_preds_;
    std::vector<float> box_preds_;
    std::vector<Detection> detections_;
    int num_pillars_;

    void accumulate_sweeps(const Frame& frame);
    void preprocess();
    void pfe_inference();
    void scatter();
    void backbone_inference();
    void postprocess();
    void transform_to_ego();
    void transform_to_global(const EgoPose& ego_pose);
};
