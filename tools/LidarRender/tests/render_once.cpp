#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include <marsim_render/marsim_render.hpp>

namespace {

struct Options {
    std::string config;
    std::string pcd;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float yaw = 0.0f;
};

[[noreturn]] void usage(const char *program) {
    std::cerr << "用法: " << program
              << " --config PATH --pcd PATH [--x X] [--y Y] [--z Z] [--yaw RAD]\n";
    std::exit(2);
}

Options parse_args(int argc, char **argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string flag = argv[index];
        if (index + 1 >= argc) {
            usage(argv[0]);
        }
        const std::string value = argv[++index];
        if (flag == "--config") {
            options.config = value;
        } else if (flag == "--pcd") {
            options.pcd = value;
        } else if (flag == "--x") {
            options.x = std::stof(value);
        } else if (flag == "--y") {
            options.y = std::stof(value);
        } else if (flag == "--z") {
            options.z = std::stof(value);
        } else if (flag == "--yaw") {
            options.yaw = std::stof(value);
        } else {
            usage(argv[0]);
        }
    }
    if (options.config.empty() || options.pcd.empty()) {
        usage(argv[0]);
    }
    return options;
}

}  // namespace

int main(int argc, char **argv) {
    const Options options = parse_args(argc, argv);

    float sensing_horizon = 10.0f;
    try {
        sensing_horizon = YAML::LoadFile(options.config)["sensing_horizon"].as<float>(10.0f);
    } catch (const std::exception &error) {
        std::cerr << "读取配置失败: " << error.what() << "\n";
        return 1;
    }

    marsim::MarsimRender render(options.config, options.pcd);
    if (!render.isReady()) {
        std::cerr << "MarsimRender 未就绪（PCD 或 OpenGL 初始化失败）\n";
        return 1;
    }

    const Eigen::Vector3f position(options.x, options.y, options.z);
    const Eigen::Quaternionf orientation(Eigen::AngleAxisf(options.yaw, Eigen::Vector3f::UnitZ()));
    pcl::PointCloud<marsim::PointType>::Ptr cloud(new pcl::PointCloud<marsim::PointType>);
    render.renderOnceInWorld(position, orientation, 0.0, cloud);

    const float limit = sensing_horizon + 0.5f;
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();
    bool valid = !cloud->points.empty();

    for (const auto &point : cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            valid = false;
            break;
        }
        const Eigen::Vector3f offset(point.x - position.x(), point.y - position.y(),
                                      point.z - position.z());
        if (offset.norm() > limit) {
            valid = false;
            break;
        }
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        min_z = std::min(min_z, point.z);
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
        max_z = std::max(max_z, point.z);
    }

    if (!valid) {
        std::cerr << "点云无效：空、含非有限值或超出 sensing_horizon + 0.5\n";
        return 1;
    }

    std::cout << "points=" << cloud->points.size() << " bbox="
              << min_x << "," << min_y << "," << min_z << "-"
              << max_x << "," << max_y << "," << max_z << std::endl;
    return 0;
}
