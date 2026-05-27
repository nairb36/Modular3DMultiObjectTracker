// Concrete Implementation of PointPillars 3D object detection model.
// PointPillars ONNX model used for inference

#pragma once

#include "detector.hpp"
#include "scene.hpp"

class PointPillarsDetector : public Detector
{
    private:
    SensorData lidar_top_data_;

    void preprocess_lidar_data();
    void pointpillars_inference();
    void postprocess_outputs();

    public:
    std::vector<Detection> detect(const Frame&) override;
};