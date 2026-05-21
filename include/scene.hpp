#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>

struct EgoPose
{
    Eigen::Vector3d translation;
    Eigen::Vector4d rotation;
};

struct SensorData
{
    std::string file;
};

struct Calibration
{
    Eigen::Vector3d translation;
    Eigen::Vector4d rotation;
};

struct Annotation
{
    std::string instance_token;
    std::string category_name;
    Eigen::Vector3d translation;
    Eigen::Vector3d size;
    Eigen::Vector4d rotation;
    double yaw;
};

struct Frame
{
    int frame_id;
    std::string sample_token;
    double timestamp;
    EgoPose ego_pose;
    std::unordered_map<std::string, SensorData> sensor_data;
    std::vector<Annotation> annotations;
};

struct Scene
{
    std::string scene_name;
    std::string description;
    std::string log_token;
    std::unordered_map<std::string, Calibration> calibration;
    std::vector<Frame> frames;

    int num_frames() const { return static_cast<int>(frames.size()); }

    static Scene from_json(const std::string& path)
    {
        std::ifstream f(path);
        nlohmann::json j = nlohmann::json::parse(f);

        Scene scene;
        scene.scene_name = j["scene_name"].get<std::string>();
        scene.description = j["description"].get<std::string>();
        scene.log_token = j["log_token"].get<std::string>();

        for (auto& [channel, cal_json] : j["calibration"].items())
        {
            Calibration cal;
            auto& ext = cal_json["extrinsic"];
            auto t = ext["translation"].get<std::vector<double>>();
            auto r = ext["rotation"].get<std::vector<double>>();
            cal.translation = Eigen::Vector3d(t[0], t[1], t[2]);
            cal.rotation = Eigen::Vector4d(r[0], r[1], r[2], r[3]);
            scene.calibration[channel] = cal;
        }

        for (auto& fj : j["frames"])
        {
            Frame frame;
            frame.frame_id = fj["frame_id"].get<int>();
            frame.sample_token = fj["sample_token"].get<std::string>();
            frame.timestamp = fj["timestamp"].get<double>() / 1e6;

            auto ep_t = fj["ego_pose"]["translation"].get<std::vector<double>>();
            auto ep_r = fj["ego_pose"]["rotation"].get<std::vector<double>>();
            frame.ego_pose.translation = Eigen::Vector3d(ep_t[0], ep_t[1], ep_t[2]);
            frame.ego_pose.rotation = Eigen::Vector4d(ep_r[0], ep_r[1], ep_r[2], ep_r[3]);

            for (auto& [channel, sd_json] : fj["sensor_data"].items())
            {
                SensorData sd;
                sd.file = sd_json["file"].get<std::string>();
                frame.sensor_data[channel] = sd;
            }

            for (auto& aj : fj["annotations"])
            {
                Annotation ann;
                ann.instance_token = aj["instance_token"].get<std::string>();
                ann.category_name = aj["category_name"].get<std::string>();
                auto at = aj["translation"].get<std::vector<double>>();
                auto as = aj["size"].get<std::vector<double>>();
                auto ar = aj["rotation"].get<std::vector<double>>();
                ann.translation = Eigen::Vector3d(at[0], at[1], at[2]);
                ann.size = Eigen::Vector3d(as[0], as[1], as[2]);
                ann.rotation = Eigen::Vector4d(ar[0], ar[1], ar[2], ar[3]);
                ann.yaw = aj["yaw"].get<double>();
                frame.annotations.push_back(std::move(ann));
            }

            scene.frames.push_back(std::move(frame));
        }

        return scene;
    }
};
