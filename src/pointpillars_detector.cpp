// Concrete Implementation of PointPillars 3D object detection model.
// PointPillars ONNX model used for inference

#include "pointpillars_detector.hpp"

PointPillarsDetector::PointPillarsDetector(const DetectorConfig& config): data_root_(config.data_root),
                                                                          tracked_categories_(config.tracked_categories)
{
    // Define ONNX 
}


std::vector<Detection> PointPillarsDetector::detect(const Frame& frame)
{
    std::vector<Detection> detections;

    lidar_data_ = frame.sensor_data.at("LIDAR_TOP");
    std::cout<<"Lidar data file: "<<lidar_data_.file<<std::endl;

    read_lidar_data();

    // preprocess_lidar_data();

    // pointpillars_inference();

    // postprocess_outputs();

    return detections;
}


void PointPillarsDetector::read_lidar_data()
{
    std::string path = data_root_ + "/" + lidar_data_.file;
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    int num_fields = 5; // x,y,z,i,ring
    int num_points = size / (num_fields * sizeof(float));
    point_cloud_.resize(num_points * num_fields);
    file.read(reinterpret_cast<char*>(point_cloud_.data()), size);

}


void PointPillarsDetector::preprocess_lidar_data()
{
    const float x_min = -51.2f;
    const float x_max = 51.2f;
    const float y_min = -51.2f;
    const float y_max = 51.2f;
    const float z_min = -5.0f;
    const float z_max = 3.0f;
    const float voxel_x = 0.2f;
    const float voxel_y = 0.2f;
    const int max_points_per_pillar = 20;
    const int max_pillars = 30000;

    range_filter(x_min, x_max, y_min, y_max, z_min, z_max);
    voxelize(x_min, x_max, y_min, y_max, voxel_x, voxel_y, max_points_per_pillar, max_pillars);
}


void PointPillarsDetector::pointpillars_inference()
{
    pfe_inference();
    scatter();
    backbone_inference();
}


void PointPillarsDetector::range_filter(float x_min, float x_max,
                                        float y_min, float y_max,
                                        float z_min, float z_max)
{
    int num_points = point_cloud_.size() / 5;
    std::vector<float> filtered;

    for (int i = 0; i < num_points; i++)
    {
        float x = point_cloud_[5 * i];
        float y = point_cloud_[5 * i + 1];
        float z = point_cloud_[5 * i + 2];

        if (x >= x_min && x <= x_max && y >= y_min && y <= y_max && z >= z_min && z <= z_max)
        {
            for (int f = 0; f < 5; f++)
                filtered.push_back(point_cloud_[5 * i + f]);
        }
    }

    point_cloud_ = filtered;
}


void PointPillarsDetector::voxelize(float x_min, float x_max,
                                    float y_min, float y_max,
                                    float voxel_x, float voxel_y,
                                    int max_points_per_pillar, int max_pillars)
{
    const int num_features = 5;
    const int grid_x_size = static_cast<int>((x_max - x_min) / voxel_x);
    const int grid_y_size = static_cast<int>((y_max - y_min) / voxel_y);

    std::unordered_map<int, int> grid_to_pillar;

    voxels_.assign(max_pillars * max_points_per_pillar * num_features, 0.0f);
    voxel_num_points_.clear();
    voxel_coords_.clear();

    int num_points = point_cloud_.size() / num_features;
    int num_pillars = 0;

    for (int i = 0; i < num_points; i++)
    {
        float x = point_cloud_[i * 5 + 0];
        float y = point_cloud_[i * 5 + 1];

        int gx = static_cast<int>((x - x_min) / voxel_x);
        int gy = static_cast<int>((y - y_min) / voxel_y);

        if (gx < 0 || gx >= grid_x_size || gy < 0 || gy >= grid_y_size)
            continue;

        int grid_key = gy * grid_x_size + gx;
        int pillar_idx;

        if (grid_to_pillar.find(grid_key) == grid_to_pillar.end())
        {
            if (num_pillars >= max_pillars)
                continue;
            pillar_idx = num_pillars;
            grid_to_pillar[grid_key] = pillar_idx;
            voxel_num_points_.push_back(0);
            voxel_coords_.insert(voxel_coords_.end(), {0, 0, gy, gx});
            num_pillars++;
        }
        else
        {
            pillar_idx = grid_to_pillar[grid_key];
        }

        int pt_count = voxel_num_points_[pillar_idx];
        if (pt_count >= max_points_per_pillar)
            continue;

        int offset = (pillar_idx * max_points_per_pillar + pt_count) * num_features;
        for (int f = 0; f < num_features; f++)
        {
            voxels_[offset + f] = point_cloud_[i * 5 + f];
        }
        voxel_num_points_[pillar_idx]++;
    }

    voxels_.resize(num_pillars * max_points_per_pillar * num_features);
}