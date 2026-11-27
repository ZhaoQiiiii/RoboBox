#include "protocol.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>

#include <json/json.h>

namespace lidarsim {
namespace {

constexpr int kMessageDeadlineMs = 2000;

bool parse_json(const std::string& text, Json::Value& root, std::string& error) {
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    std::string parse_error;
    if (!reader->parse(text.data(), text.data() + text.size(), &root, &parse_error)) {
        error = "invalid JSON: " + parse_error;
        return false;
    }
    if (!root.isObject()) {
        error = "JSON header must be an object";
        return false;
    }
    return true;
}

bool require_uint64(const Json::Value& root, const char* key, uint64_t& value,
                    std::string& error) {
    const Json::Value& item = root[key];
    if (item.isUInt64()) {
        value = item.asUInt64();
        return true;
    }
    if (item.isInt64() && item.asInt64() >= 0) {
        value = static_cast<uint64_t>(item.asInt64());
        return true;
    }
    error = std::string(key) + " must be a non-negative integer";
    return false;
}

bool require_int64(const Json::Value& root, const char* key, int64_t& value,
                   std::string& error) {
    const Json::Value& item = root[key];
    if (item.isInt64()) {
        value = item.asInt64();
        return true;
    }
    if (item.isUInt64() && item.asUInt64() <=
            static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
        value = static_cast<int64_t>(item.asUInt64());
        return true;
    }
    error = std::string(key) + " must be an integer";
    return false;
}

bool require_finite_number(const Json::Value& value, const char* key, double& number,
                           std::string& error) {
    if (!value.isNumeric()) {
        error = std::string(key) + " must be numeric";
        return false;
    }
    number = value.asDouble();
    if (!std::isfinite(number)) {
        error = std::string(key) + " must be finite";
        return false;
    }
    return true;
}

bool require_vector(const Json::Value& root, const char* key, size_t length,
                    std::vector<double>& values, std::string& error) {
    const Json::Value& array = root[key];
    if (!array.isArray() || array.size() != length) {
        error = std::string(key) + " must be an array of length " + std::to_string(length);
        return false;
    }
    values.resize(length);
    for (Json::ArrayIndex index = 0; index < array.size(); ++index) {
        if (!require_finite_number(array[index], key, values[index], error)) {
            return false;
        }
    }
    return true;
}

bool recv_exact(int fd, uint8_t* data, size_t size,
                std::chrono::steady_clock::time_point deadline, std::string& error) {
    size_t offset = 0;
    while (offset < size) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            error = "timeout";
            return false;
        }
        const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now).count();
        pollfd descriptor{};
        descriptor.fd = fd;
        descriptor.events = POLLIN;
        const int poll_result = poll(&descriptor, 1, static_cast<int>(remaining_ms));
        if (poll_result == 0) {
            error = "timeout";
            return false;
        }
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            error = std::string("poll failed: ") + std::strerror(errno);
            return false;
        }
        const ssize_t received = recv(fd, data + offset, size - offset, 0);
        if (received == 0) {
            error = "eof";
            return false;
        }
        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }
            error = std::string("recv failed: ") + std::strerror(errno);
            return false;
        }
        offset += static_cast<size_t>(received);
    }
    return true;
}

}  // namespace

bool read_message(int fd, ReceivedMessage& message, std::string& error) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kMessageDeadlineMs);

    uint8_t prefix[12];
    if (!recv_exact(fd, prefix, sizeof(prefix), deadline, error)) {
        return false;
    }
    if (std::memcmp(prefix, kMagic, 4) != 0) {
        error = "invalid magic";
        return false;
    }
    uint32_t header_length = 0;
    uint32_t payload_length = 0;
    std::memcpy(&header_length, prefix + 4, 4);
    std::memcpy(&payload_length, prefix + 8, 4);
    header_length = ntohl(header_length);
    payload_length = ntohl(payload_length);
    if (header_length > kMaxHeaderBytes) {
        error = "header too large";
        return false;
    }
    if (payload_length > kMaxPayloadBytes) {
        error = "payload too large";
        return false;
    }

    std::vector<uint8_t> header(header_length);
    if (!recv_exact(fd, header.data(), header.size(), deadline, error)) {
        return false;
    }
    message.header_json.assign(header.begin(), header.end());
    message.payload.resize(payload_length);
    if (!recv_exact(fd, message.payload.data(), message.payload.size(), deadline, error)) {
        return false;
    }
    return true;
}

bool parse_pose(const std::string& header_json, PoseMessage& pose, std::string& error) {
    try {
        Json::Value root;
        if (!parse_json(header_json, root, error)) {
            return false;
        }
        if (!root["version"].isInt() && !root["version"].isUInt()) {
            error = "version must be an integer";
            return false;
        }
        if (root["version"].asInt() != static_cast<int>(kProtocolVersion)) {
            error = "unsupported protocol version";
            return false;
        }
        if (!root["type"].isString()) {
            error = "type must be a string";
            return false;
        }
        if (root["type"].asString() != "pose") {
            error = "expected pose message";
            return false;
        }

    if (!require_uint64(root, "seq", pose.seq, error)) {
        return false;
    }
    if (!require_int64(root, "stamp_sec", pose.stamp_sec, error)) {
        return false;
    }
    if (!require_int64(root, "stamp_nsec", pose.stamp_nsec, error)) {
        return false;
    }
    if (pose.stamp_sec < 0 || pose.stamp_nsec < 0 || pose.stamp_nsec >= 1000000000) {
        error = "invalid stamp";
        return false;
    }

    std::vector<double> position;
    std::vector<double> orientation;
    if (!require_vector(root, "position", 3, position, error)) {
        return false;
    }
    if (!require_vector(root, "orientation", 4, orientation, error)) {
        return false;
    }
    pose.position = Eigen::Vector3d(position[0], position[1], position[2]);
    pose.orientation = Eigen::Quaterniond(orientation[3], orientation[0],
                                          orientation[1], orientation[2]);
    if (pose.orientation.norm() < 1e-9) {
        error = "orientation must be non-zero";
        return false;
    }
    pose.orientation.normalize();
    return true;
    } catch (const Json::Exception& exception) {
        error = std::string("JSON error: ") + exception.what();
        return false;
    }
}

std::string encode_cloud_header(const CloudFrame& frame, std::string& error) {
    if (frame.xyz_i.size() % 4 != 0) {
        error = "xyz_i must contain 4 floats per point";
        return {};
    }
    if (frame.frame_id.empty()) {
        error = "frame_id must be non-empty";
        return {};
    }
    const size_t point_count = frame.xyz_i.size() / 4;
    const size_t point_bytes = point_count * kPointBytes;
    if (point_bytes > kMaxPayloadBytes) {
        error = "cloud payload too large";
        return {};
    }

    Json::Value root;
    root["version"] = static_cast<Json::UInt>(kProtocolVersion);
    root["type"] = "cloud";
    root["frame_seq"] = static_cast<Json::UInt64>(frame.frame_seq);
    root["pose_seq"] = static_cast<Json::UInt64>(frame.pose_seq);
    root["stamp_sec"] = static_cast<Json::Int64>(frame.stamp_sec);
    root["stamp_nsec"] = static_cast<Json::Int64>(frame.stamp_nsec);
    root["frame_id"] = frame.frame_id;
    root["point_count"] = static_cast<Json::UInt64>(point_count);
    root["point_bytes"] = static_cast<Json::UInt64>(point_bytes);

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    return Json::writeString(builder, root);
}

std::vector<uint8_t> encode_message(const std::string& header_json,
                                    const std::vector<uint8_t>& payload) {
    if (header_json.size() > kMaxHeaderBytes || payload.size() > kMaxPayloadBytes) {
        return {};
    }
    std::vector<uint8_t> message(12 + header_json.size() + payload.size());
    std::memcpy(message.data(), kMagic, 4);
    const uint32_t header_length = htonl(static_cast<uint32_t>(header_json.size()));
    const uint32_t payload_length = htonl(static_cast<uint32_t>(payload.size()));
    std::memcpy(message.data() + 4, &header_length, 4);
    std::memcpy(message.data() + 8, &payload_length, 4);
    std::memcpy(message.data() + 12, header_json.data(), header_json.size());
    if (!payload.empty()) {
        std::memcpy(message.data() + 12 + header_json.size(), payload.data(), payload.size());
    }
    return message;
}

}  // namespace lidarsim
