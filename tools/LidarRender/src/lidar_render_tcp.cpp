#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include <marsim_render/marsim_render.hpp>

#include "protocol.hpp"

namespace {

using namespace lidarsim;

volatile sig_atomic_t g_signal_stop = 0;

void handle_signal(int) {
    g_signal_stop = 1;
}

struct Options {
    std::string pcd_file;
    std::string config_file;
    std::string host = "127.0.0.1";
    uint16_t pose_port = 19082;
    uint16_t cloud_port = 19083;
    std::string frame_id = "world";
};

[[noreturn]] void usage(const char *program) {
    std::cerr << "用法: " << program
              << " --pcd_file PATH --config_file PATH"
                 " [--host 127.0.0.1] [--pose_port 19082] [--cloud_port 19083]"
                 " [--frame_id world]\n";
    std::exit(2);
}

uint16_t parse_port(const std::string &value, const char *flag) {
    try {
        const int port = std::stoi(value);
        if (port < 1 || port > 65535) {
            throw std::out_of_range("port");
        }
        return static_cast<uint16_t>(port);
    } catch (const std::exception &) {
        std::cerr << flag << " 需要 1-65535 的端口: " << value << "\n";
        std::exit(2);
    }
}

Options parse_args(int argc, char **argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string flag = argv[index];
        if (index + 1 >= argc) {
            usage(argv[0]);
        }
        const std::string value = argv[++index];
        if (flag == "--pcd_file") {
            options.pcd_file = value;
        } else if (flag == "--config_file") {
            options.config_file = value;
        } else if (flag == "--host") {
            options.host = value;
        } else if (flag == "--pose_port") {
            options.pose_port = parse_port(value, "--pose_port");
        } else if (flag == "--cloud_port") {
            options.cloud_port = parse_port(value, "--cloud_port");
        } else if (flag == "--frame_id") {
            options.frame_id = value;
        } else {
            usage(argv[0]);
        }
    }
    if (options.pcd_file.empty() || options.config_file.empty() ||
        options.frame_id.empty()) {
        usage(argv[0]);
    }
    return options;
}

int create_listener(const std::string &host, uint16_t port, std::string &error) {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        error = std::string("socket failed: ") + std::strerror(errno);
        return -1;
    }
    int enabled = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
        error = "invalid host: " + host;
        close(fd);
        return -1;
    }
    if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
        error = std::string("bind failed: ") + std::strerror(errno);
        close(fd);
        return -1;
    }
    if (listen(fd, 1) != 0) {
        error = std::string("listen failed: ") + std::strerror(errno);
        close(fd);
        return -1;
    }
    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

int accept_client(int listener, const std::atomic<bool> &stop) {
    while (!stop.load()) {
        pollfd descriptor{};
        descriptor.fd = listener;
        descriptor.events = POLLIN;
        const int result = poll(&descriptor, 1, 500);
        if (result <= 0) {
            continue;
        }
        const int client = accept(listener, nullptr, nullptr);
        if (client < 0) {
            continue;
        }
        const int flags = fcntl(client, F_GETFL, 0);
        fcntl(client, F_SETFL, flags & ~O_NONBLOCK);
        timeval timeout{2, 0};
        setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        return client;
    }
    return -1;
}

bool send_all(int fd, const std::vector<uint8_t> &data, std::string &error) {
    size_t offset = 0;
    while (offset < data.size()) {
        const ssize_t sent = send(fd, data.data() + offset, data.size() - offset,
                                  MSG_NOSIGNAL);
        if (sent > 0) {
            offset += static_cast<size_t>(sent);
            continue;
        }
        if (sent < 0 && errno == EINTR) {
            continue;
        }
        error = std::string("send failed: ") + std::strerror(errno);
        return false;
    }
    return true;
}

class LidarRenderServer {
public:
    explicit LidarRenderServer(Options options)
        : options_(std::move(options)),
          render_(options_.config_file, options_.pcd_file) {}

    bool ready() const {
        return render_.isReady();
    }

    int run() {
        std::string error;
        const int pose_listener = create_listener(options_.host, options_.pose_port, error);
        if (pose_listener < 0) {
            std::cerr << "pose listener: " << error << "\n";
            return 1;
        }
        const int cloud_listener = create_listener(options_.host, options_.cloud_port, error);
        if (cloud_listener < 0) {
            std::cerr << "cloud listener: " << error << "\n";
            close(pose_listener);
            return 1;
        }
        pose_listener_ = pose_listener;
        cloud_listener_ = cloud_listener;
        std::cout << "pose listening on " << options_.host << ":" << options_.pose_port
                  << std::endl;
        std::cout << "cloud listening on " << options_.host << ":" << options_.cloud_port
                  << std::endl;

        std::thread pose_thread(&LidarRenderServer::pose_loop, this);
        std::thread cloud_thread(&LidarRenderServer::cloud_loop, this);

        double sensing_rate = 10.0;
        try {
            sensing_rate = YAML::LoadFile(options_.config_file)["sensing_rate"]
                               .as<double>(10.0);
        } catch (const std::exception &) {
            sensing_rate = 10.0;
        }
        if (!(sensing_rate > 0.0)) {
            sensing_rate = 10.0;
        }

        const auto start = std::chrono::steady_clock::now();
        auto next_tick = start;
        const auto interval = std::chrono::duration<double>(1.0 / sensing_rate);

        while (!stop_.load() && g_signal_stop == 0) {
            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(interval);
            std::this_thread::sleep_until(next_tick);
            const auto now = std::chrono::steady_clock::now();
            if (next_tick < now - interval) {
                next_tick = now + std::chrono::duration_cast<
                                       std::chrono::steady_clock::duration>(interval);
            }
            if (!cloud_connected_.load()) {
                continue;
            }

            PoseMessage pose;
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                if (!has_pose_) {
                    continue;
                }
                pose = latest_pose_;
            }

            pcl::PointCloud<marsim::PointType>::Ptr cloud(
                new pcl::PointCloud<marsim::PointType>);
            const double pattern_time =
                std::chrono::duration<double>(now - start).count();
            render_.renderOnceInWorld(pose.position.cast<float>(),
                                      Eigen::Quaternionf(pose.orientation.cast<float>()),
                                      pattern_time, cloud);

            CloudFrame frame;
            frame.frame_seq = ++frame_seq_;
            frame.pose_seq = pose.seq;
            frame.stamp_sec = pose.stamp_sec;
            frame.stamp_nsec = pose.stamp_nsec;
            frame.frame_id = options_.frame_id;
            frame.xyz_i.reserve(cloud->points.size() * 4);
            for (const auto &point : cloud->points) {
                frame.xyz_i.push_back(point.x);
                frame.xyz_i.push_back(point.y);
                frame.xyz_i.push_back(point.z);
                frame.xyz_i.push_back(point.intensity);
            }

            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                pending_frame_ = std::move(frame);
            }
            frame_cv_.notify_one();
        }

        stop_.store(true);
        close(pose_listener_);
        close(cloud_listener_);
        pose_listener_ = -1;
        cloud_listener_ = -1;
        frame_cv_.notify_all();
        if (pose_thread.joinable()) {
            pose_thread.join();
        }
        if (cloud_thread.joinable()) {
            cloud_thread.join();
        }
        return 0;
    }

private:
    void pose_loop() {
        while (!stop_.load()) {
            const int client = accept_client(pose_listener_, stop_);
            if (client < 0) {
                break;
            }
            std::cout << "pose client connected" << std::endl;
            while (!stop_.load()) {
                ReceivedMessage message;
                std::string error;
                if (!lidarsim::read_message(client, message, error)) {
                    if (error == "timeout") {
                        continue;
                    }
                    break;
                }
                PoseMessage pose;
                if (!lidarsim::parse_pose(message.header_json, pose, error)) {
                    std::cerr << "pose protocol error: " << error << "\n";
                    break;
                }
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    latest_pose_ = pose;
                    has_pose_ = true;
                }
            }
            close(client);
            std::cout << "pose client disconnected" << std::endl;
        }
    }

    void cloud_loop() {
        while (!stop_.load()) {
            const int client = accept_client(cloud_listener_, stop_);
            if (client < 0) {
                break;
            }
            cloud_connected_.store(true);
            std::cout << "cloud client connected" << std::endl;
            while (!stop_.load()) {
                CloudFrame frame;
                {
                    std::unique_lock<std::mutex> lock(frame_mutex_);
                    frame_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                        return pending_frame_.has_value() || stop_.load();
                    });
                    if (!pending_frame_.has_value()) {
                        continue;
                    }
                    frame = std::move(*pending_frame_);
                    pending_frame_.reset();
                }

                std::string error;
                const std::string header = lidarsim::encode_cloud_header(frame, error);
                if (header.empty()) {
                    std::cerr << "cloud header error: " << error << "\n";
                    continue;
                }
                std::vector<uint8_t> payload(frame.xyz_i.size() * sizeof(float));
                if (!payload.empty()) {
                    std::memcpy(payload.data(), frame.xyz_i.data(), payload.size());
                }
                const std::vector<uint8_t> message = lidarsim::encode_message(header, payload);
                if (message.empty()) {
                    std::cerr << "cloud message too large\n";
                    continue;
                }
                if (!send_all(client, message, error)) {
                    std::cerr << "cloud send error: " << error << "\n";
                    break;
                }
            }
            cloud_connected_.store(false);
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                pending_frame_.reset();
            }
            close(client);
            std::cout << "cloud client disconnected" << std::endl;
        }
    }

    Options options_;
    marsim::MarsimRender render_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> cloud_connected_{false};

    int pose_listener_ = -1;
    int cloud_listener_ = -1;

    std::mutex pose_mutex_;
    PoseMessage latest_pose_;
    bool has_pose_ = false;

    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    std::optional<CloudFrame> pending_frame_;
    uint64_t frame_seq_ = 0;
};

}  // namespace

int main(int argc, char **argv) {
    const Options options = parse_args(argc, argv);
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    try {
        LidarRenderServer server(options);
        if (!server.ready()) {
            std::cerr << "MarsimRender 未就绪（PCD 或 OpenGL 初始化失败）\n";
            return 1;
        }
        return server.run();
    } catch (const std::exception &error) {
        std::cerr << "LidarRender 启动失败: " << error.what() << "\n";
        return 1;
    }
}
