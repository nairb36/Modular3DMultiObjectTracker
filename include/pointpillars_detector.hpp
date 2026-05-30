// Concrete Implementation of PointPillars 3D object detection model.
// PointPillars ONNX model used for inference

#pragma once

#include "detector.hpp"
#include "scene.hpp"
#include <onnxruntime_cxx_api.h>

class PointPillarsDetector : public Detector
{
    public:
    std::vector<Detection> detect(const Frame&) override;
    PointPillarsDetector(const DetectorConfig& config, const Calibration& lidar_calib);

    private:
    SensorData lidar_data_;
    std::string data_root_;
    std::vector<std::string> tracked_categories_;
    Calibration lidar_calib_;
    
    Ort::Env env_;
    Ort::Session pfe_session_;
    Ort::Session backbone_session_;  

    std::vector<float> point_cloud_;
    std::vector<float> voxels_;
    std::vector<int> voxel_num_points_;
    std::vector<int> voxel_coords_;
    std::vector<float> pillar_features_;
    std::vector<float> spatial_features_;
    std::vector<float> cls_preds_;
    std::vector<float> box_preds_;
    std::vector<Detection> detections_;
    int num_pillars_;
    int feat_h_;
    int feat_w_;

    void read_lidar_data();

    void preprocess_lidar_data();
    void range_filter(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
    void voxelize(float x_min, float x_max, float y_min, float y_max, float voxel_x, float voxel_y, int max_points_per_pillar, int max_pillars);
    
    void pointpillars_inference();
    void pfe_inference();
    void scatter();
    void backbone_inference();

    void postprocess_outputs();
    void transform_to_ego();
    void transform_to_global(const EgoPose& ego_pose);
};