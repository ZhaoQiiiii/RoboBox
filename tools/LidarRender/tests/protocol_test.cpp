#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <json/json.h>

#include "protocol.hpp"

namespace {

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            std::cerr << "CHECK failed at line " << __LINE__ << ": " #condition \
                      << "\n";                                                  \
            return false;                                                       \
        }                                                                       \
    } while (0)

const char *kPoseJson =
    R"({"version":1,"type":"pose","seq":1,"stamp_sec":1,"stamp_nsec":2,)"
    R"("position":[1.0,2.0,3.0],"orientation":[0.0,0.0,0.0,2.0]})";

bool test_parse_pose_valid() {
    lidarsim::PoseMessage pose;
    std::string error;
    CHECK(lidarsim::parse_pose(kPoseJson, pose, error));
    CHECK(pose.seq == 1);
    CHECK(pose.stamp_sec == 1);
    CHECK(pose.stamp_nsec == 2);
    CHECK(std::abs(pose.position.x() - 1.0) < 1e-9);
    CHECK(std::abs(pose.position.y() - 2.0) < 1e-9);
    CHECK(std::abs(pose.position.z() - 3.0) < 1e-9);
    CHECK(std::abs(pose.orientation.w() - 1.0) < 1e-9);
    CHECK(std::abs(pose.orientation.norm() - 1.0) < 1e-9);
    return true;
}

bool test_parse_pose_invalid() {
    lidarsim::PoseMessage pose;
    std::string error;
    CHECK(!lidarsim::parse_pose(
        R"({"version":2,"type":"pose","seq":1,"stamp_sec":1,"stamp_nsec":2,)"
        R"("position":[1,2,3],"orientation":[0,0,0,1]})", pose, error));
    CHECK(!lidarsim::parse_pose(
        R"({"version":1,"type":"cloud","seq":1,"stamp_sec":1,"stamp_nsec":2,)"
        R"("position":[1,2,3],"orientation":[0,0,0,1]})", pose, error));
    CHECK(!lidarsim::parse_pose(
        R"({"version":1,"type":"pose","seq":1,"stamp_sec":1,"stamp_nsec":2,)"
        R"("position":[1,2,3],"orientation":[0,0,0,0]})", pose, error));
    CHECK(!lidarsim::parse_pose(
        R"({"version":1,"type":"pose","seq":1,"stamp_sec":1,"stamp_nsec":2,)"
        R"("position":[1,2,1e999],"orientation":[0,0,0,1]})", pose, error));
    CHECK(!lidarsim::parse_pose(
        R"({"version":1,"type":"pose","seq":1,"stamp_sec":1,"stamp_nsec":2,)"
        R"("position":[1,2,3]})", pose, error));
    CHECK(!lidarsim::parse_pose(
        R"({"version":{"a":1},"type":"pose","seq":1,"stamp_sec":1,"stamp_nsec":2,)"
        R"("position":[1,2,3],"orientation":[0,0,0,1]})", pose, error));
    CHECK(!lidarsim::parse_pose(
        R"({"type":"pose","seq":1,"stamp_sec":1,"stamp_nsec":2,)"
        R"("position":[1,2,3],"orientation":[0,0,0,1]})", pose, error));
    CHECK(!lidarsim::parse_pose(
        R"({"version":1,"type":{},"seq":1,"stamp_sec":1,"stamp_nsec":2,)"
        R"("position":[1,2,3],"orientation":[0,0,0,1]})", pose, error));
    return true;
}

bool test_encode_cloud_header() {
    lidarsim::CloudFrame frame;
    frame.frame_seq = 7;
    frame.pose_seq = 6;
    frame.stamp_sec = 5;
    frame.stamp_nsec = 4;
    frame.frame_id = "world";
    frame.xyz_i = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f};
    std::string error;
    const std::string json = lidarsim::encode_cloud_header(frame, error);
    CHECK(!json.empty());

    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value root;
    std::string parse_error;
    CHECK(reader->parse(json.data(), json.data() + json.size(), &root, &parse_error));
    CHECK(root["version"].asInt() == 1);
    CHECK(root["type"].asString() == "cloud");
    CHECK(root["frame_seq"].asUInt64() == 7);
    CHECK(root["pose_seq"].asUInt64() == 6);
    CHECK(root["stamp_sec"].asInt64() == 5);
    CHECK(root["stamp_nsec"].asInt64() == 4);
    CHECK(root["frame_id"].asString() == "world");
    CHECK(root["point_count"].asUInt64() == 2);
    CHECK(root["point_bytes"].asUInt64() == 32);

    frame.xyz_i = {1.0f, 2.0f, 3.0f};
    CHECK(lidarsim::encode_cloud_header(frame, error).empty());

    frame.xyz_i.clear();
    const std::string empty_json = lidarsim::encode_cloud_header(frame, error);
    CHECK(!empty_json.empty());
    Json::Value empty_root;
    CHECK(reader->parse(empty_json.data(), empty_json.data() + empty_json.size(),
                        &empty_root, &parse_error));
    CHECK(empty_root["point_count"].asUInt64() == 0);
    CHECK(empty_root["point_bytes"].asUInt64() == 0);
    return true;
}

bool test_message_roundtrip() {
    lidarsim::CloudFrame frame;
    frame.frame_seq = 1;
    frame.pose_seq = 1;
    frame.stamp_sec = 0;
    frame.stamp_nsec = 0;
    frame.frame_id = "world";
    frame.xyz_i = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f};
    std::string error;
    const std::string header_json = lidarsim::encode_cloud_header(frame, error);
    CHECK(!header_json.empty());

    std::vector<uint8_t> payload(frame.xyz_i.size() * sizeof(float));
    std::memcpy(payload.data(), frame.xyz_i.data(), payload.size());
    const std::vector<uint8_t> message = lidarsim::encode_message(header_json, payload);
    CHECK(!message.empty());

    int sockets[2];
    CHECK(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    const ssize_t written = write(sockets[0], message.data(), message.size());
    CHECK(written == static_cast<ssize_t>(message.size()));

    lidarsim::ReceivedMessage received;
    CHECK(lidarsim::read_message(sockets[1], received, error));
    CHECK(received.header_json == header_json);
    CHECK(received.payload == payload);

    close(sockets[0]);
    close(sockets[1]);
    return true;
}

bool test_read_message_rejects(const char *magic, uint32_t header_length,
                               uint32_t payload_length) {
    uint8_t prefix[12];
    std::memcpy(prefix, magic, 4);
    const uint32_t header_network = htonl(header_length);
    const uint32_t payload_network = htonl(payload_length);
    std::memcpy(prefix + 4, &header_network, 4);
    std::memcpy(prefix + 8, &payload_network, 4);

    int sockets[2];
    CHECK(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    CHECK(write(sockets[0], prefix, sizeof(prefix)) == static_cast<ssize_t>(sizeof(prefix)));
    lidarsim::ReceivedMessage received;
    std::string error;
    CHECK(!lidarsim::read_message(sockets[1], received, error));
    close(sockets[0]);
    close(sockets[1]);
    return true;
}

bool test_read_message_invalid_magic() {
    return test_read_message_rejects("BAD!", 2, 0);
}

bool test_read_message_oversized_header() {
    return test_read_message_rejects("LDSM", lidarsim::kMaxHeaderBytes + 1, 0);
}

bool test_read_message_oversized_payload() {
    return test_read_message_rejects("LDSM", 2, lidarsim::kMaxPayloadBytes + 1);
}

}  // namespace

int main() {
    const bool ok = test_parse_pose_valid() && test_parse_pose_invalid() &&
                    test_encode_cloud_header() && test_message_roundtrip() &&
                    test_read_message_invalid_magic() &&
                    test_read_message_oversized_header() &&
                    test_read_message_oversized_payload();
    if (!ok) {
        return 1;
    }
    std::cout << "protocol_test: all checks passed\n";
    return 0;
}
