#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lidarsim {

constexpr uint32_t kProtocolVersion = 1;
constexpr size_t kMaxHeaderBytes = 16 * 1024;
constexpr size_t kMaxPayloadBytes = 16 * 1024 * 1024;
constexpr size_t kPointBytes = 16;
constexpr char kMagic[4] = {'L', 'D', 'S', 'M'};

struct PoseMessage {
    uint64_t seq = 0;
    int64_t stamp_sec = 0;
    int64_t stamp_nsec = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct CloudFrame {
    uint64_t frame_seq = 0;
    uint64_t pose_seq = 0;
    int64_t stamp_sec = 0;
    int64_t stamp_nsec = 0;
    std::string frame_id = "world";
    std::vector<float> xyz_i;
};

struct ReceivedMessage {
    std::string header_json;
    std::vector<uint8_t> payload;
};

bool read_message(int fd, ReceivedMessage& message, std::string& error);
bool parse_pose(const std::string& header_json, PoseMessage& pose, std::string& error);
std::string encode_cloud_header(const CloudFrame& frame, std::string& error);
std::vector<uint8_t> encode_message(const std::string& header_json,
                                    const std::vector<uint8_t>& payload);

}  // namespace lidarsim
