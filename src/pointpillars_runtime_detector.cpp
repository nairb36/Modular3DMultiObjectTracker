#include "pointpillars_runtime_detector.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <iostream>

PointPillarsRuntimeDetector::PointPillarsRuntimeDetector(const DetectorConfig& config, const Calibration& lidar_calib)
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


static Eigen::Matrix4d make_transform(const Eigen::Vector4d& quat, const Eigen::Vector3d& trans)
{
    double w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0,0) = 1-2*(y*y+z*z); T(0,1) = 2*(x*y-w*z);   T(0,2) = 2*(x*z+w*y);
    T(1,0) = 2*(x*y+w*z);   T(1,1) = 1-2*(x*x+z*z); T(1,2) = 2*(y*z-w*x);
    T(2,0) = 2*(x*z-w*y);   T(2,1) = 2*(y*z+w*x);   T(2,2) = 1-2*(x*x+y*y);
    T(0,3) = trans[0]; T(1,3) = trans[1]; T(2,3) = trans[2];
    return T;
}


std::vector<Detection> PointPillarsRuntimeDetector::detect(const Frame& frame)
{
    accumulate_sweeps(frame);
    preprocess();
    pfe_inference();
    scatter();
    backbone_inference();
    postprocess();
    transform_to_ego();
    transform_to_global(frame.ego_pose);

    std::cout << "Num Detections = " << detections_.size() << std::endl;
    return detections_;
}


void PointPillarsRuntimeDetector::accumulate_sweeps(const Frame& frame)
{
    point_cloud_.clear();

    auto load_bin = [&](const std::string& file) -> std::vector<float>
    {
        std::string path = data_root_ + "/" + file;
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        std::streamsize size = f.tellg();
        f.seekg(0, std::ios::beg);
        int num_fields = 5;
        int num_points = size / (num_fields * sizeof(float));
        std::vector<float> raw(num_points * num_fields);
        f.read(reinterpret_cast<char*>(raw.data()), size);
        return raw;
    };

    const SensorData& lidar = frame.sensor_data.at("LIDAR_TOP");

    // Current keyframe: take x,y,z,intensity, set time_lag=0
    {
        auto raw = load_bin(lidar.file);
        int np = raw.size() / 5;
        for (int i = 0; i < np; i++)
        {
            point_cloud_.push_back(raw[i*5 + 0]);
            point_cloud_.push_back(raw[i*5 + 1]);
            point_cloud_.push_back(raw[i*5 + 2]);
            point_cloud_.push_back(raw[i*5 + 3]);
            point_cloud_.push_back(0.0f);
        }
    }

    // Compute current frame transforms
    Eigen::Matrix4d T_cur_lidar_to_ego = make_transform(lidar_calib_.rotation, lidar_calib_.translation);
    Eigen::Matrix4d T_cur_ego_to_global = make_transform(frame.ego_pose.rotation, frame.ego_pose.translation);
    Eigen::Matrix4d T_global_to_cur_sensor = (T_cur_ego_to_global * T_cur_lidar_to_ego).inverse();

    double cur_timestamp = frame.timestamp * 1e6;

    // Previous sweeps
    for (const auto& sweep : lidar.sweeps)
    {
        auto raw = load_bin(sweep.file);
        int np = raw.size() / 5;

        Eigen::Matrix4d T_sw_ego_to_global = make_transform(sweep.ego_pose.rotation, sweep.ego_pose.translation);
        Eigen::Matrix4d T_sw_lidar_to_ego = T_cur_lidar_to_ego; // calibration is constant per scene
        Eigen::Matrix4d T = T_global_to_cur_sensor * T_sw_ego_to_global * T_sw_lidar_to_ego;

        float time_lag = static_cast<float>((cur_timestamp - sweep.timestamp) / 1e6);

        for (int i = 0; i < np; i++)
        {
            float x = raw[i*5 + 0];
            float y = raw[i*5 + 1];
            float z = raw[i*5 + 2];
            float intensity = raw[i*5 + 3];

            // Remove ego-vehicle points
            if (std::abs(x) < 1.0f && std::abs(y) < 1.0f)
                continue;

            Eigen::Vector4d pt(x, y, z, 1.0);
            Eigen::Vector4d pt_cur = T * pt;

            point_cloud_.push_back(static_cast<float>(pt_cur[0]));
            point_cloud_.push_back(static_cast<float>(pt_cur[1]));
            point_cloud_.push_back(static_cast<float>(pt_cur[2]));
            point_cloud_.push_back(intensity);
            point_cloud_.push_back(time_lag);
        }
    }

    std::cout << "Accumulated " << point_cloud_.size() / 5 << " points from "
              << 1 + lidar.sweeps.size() << " sweeps" << std::endl;
}


void PointPillarsRuntimeDetector::preprocess()
{
    const float x_min = -51.2f, x_max = 51.2f;
    const float y_min = -51.2f, y_max = 51.2f;
    const float z_min = -5.0f, z_max = 3.0f;
    const float voxel_x = 0.2f, voxel_y = 0.2f;
    const int max_points_per_pillar = 20;
    const int max_pillars = 30000;
    const int num_features = 5;
    const int grid_x_size = 512;
    const int grid_y_size = 512;

    // Range filter + voxelize in one pass
    std::unordered_map<int, int> grid_to_pillar;
    voxels_.assign(max_pillars * max_points_per_pillar * num_features, 0.0f);
    voxel_num_points_.clear();
    voxel_coords_.clear();

    int num_points = point_cloud_.size() / num_features;
    int num_pillars = 0;

    for (int i = 0; i < num_points; i++)
    {
        float x = point_cloud_[i*5 + 0];
        float y = point_cloud_[i*5 + 1];
        float z = point_cloud_[i*5 + 2];

        if (x < x_min || x > x_max || y < y_min || y > y_max || z < z_min || z > z_max)
            continue;

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
            voxels_[offset + f] = point_cloud_[i*5 + f];
        voxel_num_points_[pillar_idx]++;
    }

    voxels_.resize(num_pillars * max_points_per_pillar * num_features);
    num_pillars_ = num_pillars;
}


void PointPillarsRuntimeDetector::pfe_inference()
{
    const int max_points_per_pillar = 20;
    const int num_features = 5;

    std::vector<int64_t> voxels_shape = {num_pillars_, max_points_per_pillar, num_features};
    std::vector<int64_t> num_points_shape = {num_pillars_};
    std::vector<int64_t> coords_shape = {num_pillars_, 4};

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value voxels_tensor = Ort::Value::CreateTensor<float>(
        memory_info, voxels_.data(), voxels_.size(), voxels_shape.data(), voxels_shape.size());

    Ort::Value num_points_tensor = Ort::Value::CreateTensor<int64_t>(
        memory_info, voxel_num_points_.data(), voxel_num_points_.size(),
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


void PointPillarsRuntimeDetector::scatter()
{
    const int C = 64, H = 512, W = 512;
    spatial_features_.assign(C * H * W, 0.0f);

    for (int p = 0; p < num_pillars_; p++)
    {
        int gy = voxel_coords_[p * 4 + 2];
        int gx = voxel_coords_[p * 4 + 3];

        for (int c = 0; c < C; c++)
            spatial_features_[c * H * W + gy * W + gx] = pillar_features_[p * C + c];
    }
}


void PointPillarsRuntimeDetector::backbone_inference()
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
}


void PointPillarsRuntimeDetector::postprocess()
{
    const float score_threshold = 0.1f;
    const float nms_iou_threshold = 0.2f;
    const int nms_pre_maxsize = 1000;
    const int nms_post_maxsize = 83;
    const int H = 128, W = 128;
    const int box_code_size = 10;

    const float x_stride = 102.4f / 127.0f;
    const float y_stride = 102.4f / 127.0f;

    struct AnchorDef {
        std::string name;
        float dx, dy, dz, z_center;
        float rotation;
    };

    struct HeadDef {
        int num_classes;
        int num_anchors;
        std::vector<std::string> class_names;
        std::vector<AnchorDef> anchors;
    };

    auto make_head = [](const std::vector<std::tuple<std::string, float, float, float, float>>& classes) -> HeadDef
    {
        HeadDef head;
        head.num_classes = classes.size();
        head.num_anchors = classes.size() * 2;
        for (auto& [name, dx, dy, dz, zc] : classes)
        {
            head.class_names.push_back(name);
            head.anchors.push_back({name, dx, dy, dz, zc, 0.0f});
            head.anchors.push_back({name, dx, dy, dz, zc, 1.5708f});
        }
        return head;
    };

    std::vector<HeadDef> heads = {
        make_head({{"car",                  4.63f, 1.97f, 1.74f, -0.0800f}}),
        make_head({{"truck",                6.93f, 2.51f, 2.84f,  0.8200f},
                   {"construction_vehicle", 6.37f, 2.85f, 3.19f,  1.3700f}}),
        make_head({{"bus",                  10.5f, 2.94f, 3.47f,  1.6500f},
                   {"trailer",              12.29f,2.90f, 3.87f,  2.0500f}}),
        make_head({{"barrier",              0.50f, 2.53f, 0.98f, -0.8400f}}),
        make_head({{"motorcycle",           2.11f, 0.77f, 1.47f, -0.3500f},
                   {"bicycle",              1.70f, 0.60f, 1.28f, -0.5400f}}),
        make_head({{"pedestrian",           0.73f, 0.67f, 1.77f, -0.0500f},
                   {"traffic_cone",         0.41f, 0.41f, 1.07f, -0.7500f}}),
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
        float x, y, z, dx, dy, dz, yaw, vx, vy, score;
        std::string category;
    };

    // BEV NMS helpers
    struct Pt { float x, y; };

    auto get_corners = [](const RawDet& d, Pt out[4]) {
        float c = std::cos(d.yaw), s = std::sin(d.yaw);
        float hl = d.dx / 2, hw = d.dy / 2;
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
        float area_a = a.dx * a.dy, area_b = b.dx * b.dy;
        return inter / (area_a + area_b - inter + 1e-6f);
    };

    detections_.clear();

    int cls_offset = 0;
    int box_offset = 0;

    for (auto& head : heads)
    {
        int na = head.num_anchors;
        int nc = head.num_classes;
        int cls_channels = na * nc;
        int box_channels = na * box_code_size;

        // Per-class NMS within each head
        for (int c = 0; c < nc; c++)
        {
            std::vector<RawDet> class_dets;

            for (int a = 0; a < na; a++)
            {
                int anchor_class = a / 2;
                if (anchor_class != c)
                    continue;

                auto& anchor = head.anchors[a];
                float diag = std::sqrt(anchor.dx * anchor.dx + anchor.dy * anchor.dy);

                for (int gy = 0; gy < H; gy++)
                {
                    for (int gx = 0; gx < W; gx++)
                    {
                        int cls_ch_idx = cls_offset + a * nc + c;
                        float raw_score = cls_preds_[cls_ch_idx * H * W + gy * W + gx];
                        float score = 1.0f / (1.0f + std::exp(-raw_score));

                        if (score <= score_threshold)
                            continue;

                        auto ch = [&](int idx) {
                            return box_preds_[idx * H * W + gy * W + gx];
                        };

                        int base = box_offset + a * box_code_size;
                        float xt = ch(base + 0);
                        float yt = ch(base + 1);
                        float zt = ch(base + 2);
                        float log_dxt = ch(base + 3);
                        float log_dyt = ch(base + 4);
                        float log_dzt = ch(base + 5);
                        float cos_t = ch(base + 6);
                        float sin_t = ch(base + 7);
                        float vxt = ch(base + 8);
                        float vyt = ch(base + 9);

                        float ax = -51.2f + gx * x_stride;
                        float ay = -51.2f + gy * y_stride;

                        RawDet det;
                        det.x = xt * diag + ax;
                        det.y = yt * diag + ay;
                        det.z = zt * anchor.dz + anchor.z_center;
                        det.dx = std::exp(log_dxt) * anchor.dx;
                        det.dy = std::exp(log_dyt) * anchor.dy;
                        det.dz = std::exp(log_dzt) * anchor.dz;
                        det.yaw = std::atan2(sin_t + std::sin(anchor.rotation),
                                             cos_t + std::cos(anchor.rotation));
                        det.vx = vxt;
                        det.vy = vyt;
                        det.score = score;
                        det.category = class_to_category[head.class_names[c]];

                        class_dets.push_back(det);
                    }
                }
            }

            // Sort by score descending
            std::sort(class_dets.begin(), class_dets.end(),
                [](const RawDet& a, const RawDet& b) { return a.score > b.score; });

            // Pre-max truncation
            if ((int)class_dets.size() > nms_pre_maxsize)
                class_dets.resize(nms_pre_maxsize);

            // NMS
            std::vector<bool> suppressed(class_dets.size(), false);
            int kept = 0;

            for (size_t i = 0; i < class_dets.size() && kept < nms_post_maxsize; i++)
            {
                if (suppressed[i]) continue;

                bool tracked = false;
                for (auto& cat : tracked_categories_)
                {
                    if (class_dets[i].category.find(cat) != std::string::npos)
                    {
                        tracked = true;
                        break;
                    }
                }

                if (tracked)
                {
                    Detection d;
                    d.category_name_ = class_dets[i].category;
                    d.confidence_ = class_dets[i].score;
                    d.position_ = Eigen::Vector3d(class_dets[i].x, class_dets[i].y, class_dets[i].z);
                    d.bbox_dims_ = Eigen::Vector3d(class_dets[i].dy, class_dets[i].dx, class_dets[i].dz);
                    d.yaw_ = class_dets[i].yaw;
                    d.rotation_quaternion_ = Eigen::Vector4d(
                        std::cos(class_dets[i].yaw / 2), 0, 0, std::sin(class_dets[i].yaw / 2));
                    detections_.push_back(d);
                }

                kept++;

                for (size_t j = i + 1; j < class_dets.size(); j++)
                {
                    if (suppressed[j]) continue;
                    if (iou_bev(class_dets[i], class_dets[j]) > nms_iou_threshold)
                        suppressed[j] = true;
                }
            }
        }

        cls_offset += cls_channels;
        box_offset += box_channels;
    }
}


void PointPillarsRuntimeDetector::transform_to_ego()
{
    Eigen::Matrix3d R_calib = make_transform(lidar_calib_.rotation, lidar_calib_.translation).block<3,3>(0,0);
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


void PointPillarsRuntimeDetector::transform_to_global(const EgoPose& ego_pose)
{
    Eigen::Matrix3d R_ego = make_transform(ego_pose.rotation, ego_pose.translation).block<3,3>(0,0);
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
