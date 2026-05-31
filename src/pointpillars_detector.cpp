// Concrete Implementation of PointPillars 3D object detection model.
// PointPillars ONNX model used for inference

#include "pointpillars_detector.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <iostream>

PointPillarsDetector::PointPillarsDetector(const DetectorConfig& config, const Calibration& lidar_calib)
    : data_root_(config.data_root),
      tracked_categories_(config.tracked_categories),
      lidar_calib_(lidar_calib),
      env_(ORT_LOGGING_LEVEL_WARNING, "pointpillars"),
      pfe_session_(env_, "/workspace/src/Project_MOT/models/pointpillars/pointpillars_pfe.onnx", [](){
          Ort::SessionOptions opts;
          opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
          return opts;}()),
      backbone_session_(env_, "/workspace/src/Project_MOT/models/pointpillars/pointpillars_backbone.onnx", [](){
          Ort::SessionOptions opts;
          opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
          return opts;}())
{
}


std::vector<Detection> PointPillarsDetector::detect(const Frame& frame)
{
    std::vector<Detection> detections;

    lidar_data_ = frame.sensor_data.at("LIDAR_TOP");
    std::cout<<"Lidar data file: "<<lidar_data_.file<<std::endl;

    read_lidar_data();
    preprocess_lidar_data();
    pointpillars_inference();
    postprocess_outputs();
    transform_to_ego();
    transform_to_global(frame.ego_pose);

    std::cout<<"Num Detections = "<<detections_.size()<<std::endl;
    return detections_;
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


void PointPillarsDetector::pfe_inference()
{
    const int max_points_per_pillar = 20;
    const int num_features = 5;
    num_pillars_ = voxel_num_points_.size();

    std::vector<int64_t> voxels_shape = {num_pillars_, max_points_per_pillar, num_features};
    std::vector<int64_t> num_points_shape = {num_pillars_};
    std::vector<int64_t> coords_shape = {num_pillars_, 4};

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    std::vector<int64_t> voxel_num_points_i64(voxel_num_points_.begin(), voxel_num_points_.end());

    Ort::Value voxels_tensor = Ort::Value::CreateTensor<float>(
        memory_info, voxels_.data(), voxels_.size(), voxels_shape.data(), voxels_shape.size());

    Ort::Value num_points_tensor = Ort::Value::CreateTensor<int64_t>(
        memory_info, voxel_num_points_i64.data(), voxel_num_points_i64.size(),
        num_points_shape.data(), num_points_shape.size());

    Ort::Value coords_tensor = Ort::Value::CreateTensor<int>(
        memory_info, voxel_coords_.data(), voxel_coords_.size(),
        coords_shape.data(), coords_shape.size());

    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(std::move(voxels_tensor));
    input_tensors.push_back(std::move(num_points_tensor));
    input_tensors.push_back(std::move(coords_tensor));

    const char* input_names[] = {"voxels", "voxel_num_points", "voxel_coords"};
    const char* output_names[] = {"pillar_features"};

    auto outputs = pfe_session_.Run(Ort::RunOptions{nullptr},
        input_names, input_tensors.data(), 3,
        output_names, 1);

    auto& out_tensor = outputs[0];
    auto out_shape = out_tensor.GetTensorTypeAndShapeInfo().GetShape();
    int64_t total = out_shape[0] * out_shape[1];

    float* out_data = out_tensor.GetTensorMutableData<float>();
    pillar_features_.assign(out_data, out_data + total);
}


void PointPillarsDetector::scatter()
{
    const int C = 64;
    const int H = 512;
    const int W = 512;

    spatial_features_.assign(1 * C * H * W, 0.0f);

    for (int p = 0; p < num_pillars_; p++)
    {
        int gy = voxel_coords_[p * 4 + 2];
        int gx = voxel_coords_[p * 4 + 3];

        for (int c = 0; c < C; c++)
        {
            spatial_features_[c * H * W + gy * W + gx] = pillar_features_[p * C + c];
        }
    }
}


void PointPillarsDetector::backbone_inference()
{
    const int C = 64, H = 512, W = 512;
    std::vector<int64_t> input_shape = {1, C, H, W};

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, spatial_features_.data(), spatial_features_.size(),
        input_shape.data(), input_shape.size());

    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(std::move(input_tensor));

    const char* input_names[] = {"spatial_features"};
    const char* output_names[] = {"cls_preds", "box_preds"};

    auto outputs = backbone_session_.Run(Ort::RunOptions{nullptr},
        input_names, input_tensors.data(), 1,
        output_names, 2);

    auto cls_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    auto box_shape = outputs[1].GetTensorTypeAndShapeInfo().GetShape();

    float* cls_data = outputs[0].GetTensorMutableData<float>();
    float* box_data = outputs[1].GetTensorMutableData<float>();

    int64_t cls_total = cls_shape[1] * cls_shape[2] * cls_shape[3];
    int64_t box_total = box_shape[1] * box_shape[2] * box_shape[3];

    cls_preds_.assign(cls_data, cls_data + cls_total);
    box_preds_.assign(box_data, box_data + box_total);

    feat_h_ = cls_shape[2];
    feat_w_ = cls_shape[3];
}


void PointPillarsDetector::postprocess_outputs()
{
    const float score_threshold = 0.3f;
    const float nms_threshold = 0.2f;
    const float x_min = -51.2f, y_min = -51.2f;
    const float pixel_size = 0.8f;
    const int box_code_size = 10;

    struct AnchorDef {
        std::string name;
        float l, w, h, z;
        float rotation;
    };

    struct HeadDef {
        int cls_channels;
        int num_anchors;
        int num_classes;
        std::vector<std::string> class_names;
        std::vector<AnchorDef> anchors;
    };

    auto make_anchors = [](const std::vector<std::pair<std::string, std::array<float,4>>>& classes) -> HeadDef
    {
        HeadDef head;
        head.num_classes = classes.size();
        head.num_anchors = classes.size() * 2;
        head.cls_channels = head.num_anchors * head.num_classes;
        for (auto& [name, dims] : classes)
        {
            head.class_names.push_back(name);
            head.anchors.push_back({name, dims[0], dims[1], dims[2], dims[3], 0.0f});
            head.anchors.push_back({name, dims[0], dims[1], dims[2], dims[3], 1.5708f});
        }
        return head;
    };

    std::vector<HeadDef> heads = {
        make_anchors({{"car",                  {4.63f, 1.97f, 1.74f, -0.95f}}}),
        make_anchors({{"truck",                {6.93f, 2.51f, 2.84f, -0.40f}},
                      {"construction_vehicle", {6.37f, 2.85f, 3.19f, -0.225f}}}),
        make_anchors({{"bus",                  {10.5f, 2.94f, 3.47f, -0.085f}},
                      {"trailer",              {12.29f,2.90f, 3.87f, -0.26f}}}),
        make_anchors({{"barrier",              {0.50f, 2.53f, 0.98f, -1.67f}}}),
        make_anchors({{"motorcycle",           {2.11f, 0.77f, 1.47f, -1.03f}},
                      {"bicycle",              {1.70f, 0.60f, 1.28f, -1.04f}}}),
        make_anchors({{"pedestrian",           {0.73f, 0.67f, 1.77f, -0.73f}},
                      {"traffic_cone",         {0.41f, 0.41f, 1.07f, -1.78f}}}),
    };

    std::unordered_map<std::string, std::string> class_to_category = {
        {"car", "vehicle.car"},
        {"truck", "vehicle.truck"},
        {"construction_vehicle", "vehicle.construction"},
        {"bus", "vehicle.bus"},
        {"trailer", "vehicle.trailer"},
        {"barrier", "movable_object.barrier"},
        {"motorcycle", "vehicle.motorcycle"},
        {"bicycle", "vehicle.bicycle"},
        {"pedestrian", "human.pedestrian"},
        {"traffic_cone", "movable_object.trafficcone"},
    };

    struct RawDet {
        float x, y, z, l, w, h, yaw, score;
        std::string category;
    };

    std::vector<RawDet> all_dets;

    int cls_offset = 0;
    int box_offset = 0;
    int H = feat_h_, W = feat_w_;

    for (auto& head : heads)
    {
        int na = head.num_anchors;
        int nc = head.num_classes;
        int box_ch = na * box_code_size;

        for (int gy = 0; gy < H; gy++)
        {
            for (int gx = 0; gx < W; gx++)
            {
                for (int a = 0; a < na; a++)
                {
                    float max_score = -1e9f;
                    int best_cls = 0;
                    for (int c = 0; c < nc; c++)
                    {
                        int cls_ch_idx = cls_offset + a * nc + c;
                        float raw = cls_preds_[cls_ch_idx * H * W + gy * W + gx];
                        float sig = 1.0f / (1.0f + std::exp(-raw));
                        if (sig > max_score)
                        {
                            max_score = sig;
                            best_cls = c;
                        }
                    }

                    if (max_score < score_threshold)
                        continue;

                    auto ch = [&](int c) {
                        return box_preds_[c * H * W + gy * W + gx];
                    };

                    int base = box_offset + a * box_code_size;
                    float dx = ch(base + 0);
                    float dy = ch(base + 1);
                    float dz = ch(base + 2);
                    float dl = ch(base + 3);
                    float dw = ch(base + 4);
                    float dh = ch(base + 5);
                    float sin_yaw = ch(base + 6);
                    float cos_yaw = ch(base + 7);

                    auto& anchor = head.anchors[a];
                    float diag = std::sqrt(anchor.l * anchor.l + anchor.w * anchor.w);

                    float ax = x_min + (gx + 0.5f) * pixel_size;
                    float ay = y_min + (gy + 0.5f) * pixel_size;

                    RawDet det;
                    det.x = ax + dx * diag;
                    det.y = ay + dy * diag;
                    det.z = anchor.z + anchor.h / 2 + dz * anchor.h;
                    det.l = anchor.l * std::exp(dl);
                    det.w = anchor.w * std::exp(dw);
                    det.h = anchor.h * std::exp(dh);
                    det.yaw = std::atan2(sin_yaw, cos_yaw) + anchor.rotation;
                    det.score = max_score;

                    int class_per_ori = a / 2;
                    det.category = class_to_category[head.class_names[class_per_ori]];

                    all_dets.push_back(det);
                }
            }
        }
        cls_offset += head.cls_channels;
        box_offset += box_ch;
    }

    // Rotated BEV NMS
    struct Pt { float x, y; };

    auto get_corners = [](const RawDet& d, Pt out[4]) {
        float c = std::cos(d.yaw), s = std::sin(d.yaw);
        float hl = d.l / 2, hw = d.w / 2;
        out[0] = {d.x + hl*c - hw*s, d.y + hl*s + hw*c};
        out[1] = {d.x - hl*c - hw*s, d.y - hl*s + hw*c};
        out[2] = {d.x - hl*c + hw*s, d.y - hl*s - hw*c};
        out[3] = {d.x + hl*c + hw*s, d.y + hl*s - hw*c};
    };

    auto cross2d = [](Pt o, Pt a, Pt b) {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };

    auto seg_isect = [](Pt a, Pt b, Pt c, Pt d) -> Pt {
        float ax = b.x-a.x, ay = b.y-a.y, cx = d.x-c.x, cy = d.y-c.y;
        float t = ((c.x-a.x)*cy - (c.y-a.y)*cx) / (ax*cy - ay*cx);
        return {a.x + t*ax, a.y + t*ay};
    };

    auto poly_area = [](const std::vector<Pt>& p) -> float {
        float a = 0;
        int n = p.size();
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            a += p[i].x * p[j].y - p[j].x * p[i].y;
        }
        return std::abs(a) / 2;
    };

    auto iou_bev = [&](const RawDet& a, const RawDet& b) -> float {
        Pt ca[4], cb[4];
        get_corners(a, ca);
        get_corners(b, cb);

        std::vector<Pt> poly(cb, cb + 4);
        for (int i = 0; i < 4 && !poly.empty(); i++) {
            std::vector<Pt> input = poly;
            poly.clear();
            Pt e0 = ca[i], e1 = ca[(i + 1) % 4];
            for (int j = 0, n = input.size(); j < n; j++) {
                Pt cur = input[j], prv = input[(j + n - 1) % n];
                float cc = cross2d(e0, e1, cur), cp = cross2d(e0, e1, prv);
                if (cc >= 0) {
                    if (cp < 0) poly.push_back(seg_isect(prv, cur, e0, e1));
                    poly.push_back(cur);
                } else if (cp >= 0) {
                    poly.push_back(seg_isect(prv, cur, e0, e1));
                }
            }
        }

        float inter = poly_area(poly);
        float area_a = a.l * a.w, area_b = b.l * b.w;
        return inter / (area_a + area_b - inter + 1e-6f);
    };

    std::sort(all_dets.begin(), all_dets.end(),
        [](const RawDet& a, const RawDet& b) { return a.score > b.score; });

    std::vector<bool> suppressed(all_dets.size(), false);

    detections_.clear();
    for (size_t i = 0; i < all_dets.size(); i++)
    {
        if (suppressed[i]) continue;

        bool tracked = false;
        for (auto& cat : tracked_categories_)
        {
            if (all_dets[i].category.find(cat) != std::string::npos)
            {
                tracked = true;
                break;
            }
        }
        if (!tracked) continue;

        Detection d;
        d.category_name_ = all_dets[i].category;
        d.confidence_ = all_dets[i].score;
        d.position_ = Eigen::Vector3d(all_dets[i].x, all_dets[i].y, all_dets[i].z);
        d.bbox_dims_ = Eigen::Vector3d(all_dets[i].w, all_dets[i].l, all_dets[i].h);
        d.yaw_ = all_dets[i].yaw;
        d.rotation_quaternion_ = Eigen::Vector4d(
            std::cos(all_dets[i].yaw / 2), 0, 0, std::sin(all_dets[i].yaw / 2));
        detections_.push_back(d);

        for (size_t j = i + 1; j < all_dets.size(); j++)
        {
            if (suppressed[j]) continue;
            if (all_dets[j].category != all_dets[i].category) continue;
            if (iou_bev(all_dets[i], all_dets[j]) > nms_threshold)
                suppressed[j] = true;
        }
    }
}


void PointPillarsDetector::transform_to_ego()
{
    auto quat_to_matrix = [](const Eigen::Vector4d& q) -> Eigen::Matrix3d
    {
        double w = q[0], x = q[1], y = q[2], z = q[3];
        Eigen::Matrix3d R;
        R << 1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
             2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
             2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y);
        return R;
    };

    Eigen::Matrix3d R_calib = quat_to_matrix(lidar_calib_.rotation);
    Eigen::Vector3d t_calib = lidar_calib_.translation;

    for (auto& det : detections_)
    {
        det.position_ = R_calib * det.position_ + t_calib;

        Eigen::Vector3d heading(std::cos(det.yaw_), std::sin(det.yaw_), 0.0);
        Eigen::Vector3d heading_ego = R_calib * heading;
        det.yaw_ = std::atan2(heading_ego[1], heading_ego[0]);
        det.rotation_quaternion_ = Eigen::Vector4d(
            std::cos(det.yaw_ / 2), 0, 0, std::sin(det.yaw_ / 2));
    }
}


void PointPillarsDetector::transform_to_global(const EgoPose& ego_pose)
{
    auto quat_to_matrix = [](const Eigen::Vector4d& q) -> Eigen::Matrix3d
    {
        double w = q[0], x = q[1], y = q[2], z = q[3];
        Eigen::Matrix3d R;
        R << 1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
             2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
             2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y);
        return R;
    };

    Eigen::Matrix3d R_ego = quat_to_matrix(ego_pose.rotation);
    Eigen::Vector3d t_ego = ego_pose.translation;

    for (auto& det : detections_)
    {
        det.position_ = R_ego * det.position_ + t_ego;

        Eigen::Vector3d heading(std::cos(det.yaw_), std::sin(det.yaw_), 0.0);
        Eigen::Vector3d heading_global = R_ego * heading;
        det.yaw_ = std::atan2(heading_global[1], heading_global[0]);
        det.rotation_quaternion_ = Eigen::Vector4d(
            std::cos(det.yaw_ / 2), 0, 0, std::sin(det.yaw_ / 2));
    }
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
        voxels_[offset + 0] = point_cloud_[i * 5 + 0];
        voxels_[offset + 1] = point_cloud_[i * 5 + 1];
        voxels_[offset + 2] = point_cloud_[i * 5 + 2];
        voxels_[offset + 3] = point_cloud_[i * 5 + 3] / 255.0f;
        voxels_[offset + 4] = 0.0f;
        voxel_num_points_[pillar_idx]++;
    }

    voxels_.resize(num_pillars * max_points_per_pillar * num_features);
}