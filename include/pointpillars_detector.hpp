// Concrete Implementation of PointPillars 3D object detection model.
// PointPillars ONNX model used for inference

#pragma once

#include "detector.hpp"
#include "scene.hpp"

class PointPillarsDetector : public Detector
{
    public:
    std::vector<Detection> detect(const Frame&) override;
    PointPillarsDetector(const DetectorConfig& config);

    private:
    SensorData lidar_data_;
    std::string data_root_;
    std::vector<float> point_cloud_;
    std::vector<float> voxels_;
    std::vector<int> voxel_num_points_;
    std::vector<int> voxel_coords_;

    void read_lidar_data();
    void preprocess_lidar_data();
    void range_filter(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
    void voxelize(float x_min, float x_max, float y_min, float y_max, float voxel_x, float voxel_y, int max_points_per_pillar, int max_pillars);
    void pfe_inference();
    void scatter();
    void backbone_inference();
    void postprocess_outputs();
};