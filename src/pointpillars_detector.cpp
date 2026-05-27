// Concrete Implementation of PointPillars 3D object detection model.
// PointPillars ONNX model used for inference

#include "pointpillars_detector.hpp"


std::vector<Detection> PointPillarsDetector::detect(const Frame& frame)
{
    std::vector<Detection> detections;

    lidar_top_data_ = frame.sensor_data.at("LIDAR_TOP");
    std::cout<<"Lidar data file: "<<lidar_top_data.file<<std::endl;

    // void preprocess_lidar_data();

    // void pointpillars_inference();

    // void postprocess_outputs();

    return detections;
}


void PointPillarsDetector::preprocess_lidar_data()
{

}