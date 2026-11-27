# LidarRender

基于 SUPER `marsim_render` 的 PCD 激光雷达仿真工具。输入一张 `.pcd` 世界地图和 odom 位姿，
输出局部扫描点云与全局点云，经 ROS 1 `PointCloud2` 发布。

## 架构

```
ROS odom ──► lidar_ros_bridge.py ──TCP pose──► lidar_render_tcp (C++)
                    ▲                                │
                    │                          MarsimRender
                    │                                │
                    └──TCP cloud frame───────────────┘
                    │
                    ├─► /super/sim/cloud_registered  (local, world)
                    └─► /super/sim/global_pc         (full PCD, on subscriber)
```

- `lidar_render_tcp`：C++/GLFW/OpenGL，原样复用 `marsim_render/`；主线程渲染，位姿线程与
  发送线程通过容量 1 的帧槽解耦。
- `lidar_ros_bridge.py`：ROS 1 进程，订阅 odom、发 pose、收 cloud frame，发布
  `sensor_msgs/PointCloud2`；全局云由它读取本地 PCD 发布，不经过 TCP。

## 端口与话题

| 方向 | 默认 | 说明 |
|---|---|---|
| TCP | `19082` | pose（bridge → renderer） |
| TCP | `19083` | cloud（renderer → bridge） |
| 订阅 | `/super/sim/lidar_slam/odom` | `nav_msgs/Odometry` |
| 发布 | `/super/sim/cloud_registered` | 局部扫描，`frame_id=world` |
| 发布 | `/super/sim/global_pc` | 整张 PCD，订阅者出现时发布一次 |

点云字段固定 `x/y/z/intensity`，`FLOAT32`，`point_step=16`，`height=1`。

## 构建与运行

镜像由 `runtime/RoboKit/docker/Dockerfile` 统一构建为 `robokit:latest`（同时包含
GsplatRender）：

```bash
bash runtime/RoboKit/tools/LidarRender/scripts/build.sh
```

主仓库 `scripts/run.sh` 在 `--robot drone` 且 `SCENE_PCD` 非空时，会在统一的 `robokit`
容器中同时启动 GsplatRender 与 LidarRender（端口 `19082/19083`），并让 SUPER 仿真器只发
odom/TF、由 LidarRender 接管 `/super/sim/cloud_registered` 与 `/super/sim/global_pc`。

手动运行（需要宿主 X11；ROS master 由外部提供）：

```bash
export ROS_MASTER_URI=http://localhost:11311
bash runtime/RoboKit/tools/LidarRender/scripts/run.sh /path/to/scene.pcd
```

后台运行与停止：

```bash
bash runtime/RoboKit/tools/LidarRender/scripts/run.sh --detach --name lidar-render /path/to/scene.pcd
docker logs -f lidar-render
bash runtime/RoboKit/tools/LidarRender/scripts/stop.sh lidar-render
```

无宿主 X11 时可用软件渲染（性能明显下降）：

```bash
bash runtime/RoboKit/tools/LidarRender/scripts/run.sh --use-xvfb /path/to/scene.pcd
```

### 与 SUPER 的关系

LidarRender 只负责点云感知；odom/TF 仍由现有仿真器或其他定位源提供。例如让
`perfect_drone_sim` 只发 odom/TF（PCD 不可用时它会跳过点云发布），LidarRender 订阅同一个
odom 话题并发布点云。不要让 `perfect_drone_sim` 和 LidarRender 同时发布
`/super/sim/cloud_registered`。

## 配置

`config/params.yaml` 是 `marsim_render` 的感知参数。默认 `lidar_type: 3`（MID_360）、
`polar_resolution: 0.2`、`vertical_fov: 105`、`sensing_rate: 10`。MID_360 的垂直下限
约 -7°，低空贴地时 10 m 内可能看不到地面；需要覆盖更大垂直范围时改用
`lidar_type: 2`（GENERAL_360）。

## PCD 支持

全局点云由 bridge 解析 PCD 的 `DATA binary`，要求 `FIELDS` 含 `x y z`，`intensity`
可选（缺失时补 0）。`ascii` / `binary_compressed` 会记录警告并只发布局部云；局部云由
renderer 用 PCL 加载，支持 PCL 能读的 PCD 格式。

## 测试

```bash
# C++ protocol
cd build && ctest --output-on-failure

# 纯 Python
PYTHONPATH=runtime python3 -m unittest discover -s tests -p 'test_tcp_protocol.py' -v
PYTHONPATH=runtime python3 -m unittest discover -s tests -p 'test_pcd_reader.py' -v
PYTHONPATH=runtime python3 -m unittest discover -s tests -p 'test_bridge_helpers.py' -v

# ROS 相关（容器内）
source /opt/ros/noetic/setup.bash
PYTHONPATH=runtime:${PYTHONPATH:-} /usr/bin/python3 -m unittest discover -s tests -p 'test_cloud2.py' -v
PYTHONPATH=runtime python3 -m unittest discover -s tests -p 'test_renderer_tcp.py' -v
```

端到端冒烟：`tests/smoke_test.sh`（需要 `zero-agent-sim` 内已有 roscore）。

## 许可

`marsim_render/` 复制自 SUPER，保留 LGPL-3.0；见 `README-SOURCE.md`。RoboKit 自有代码
沿用仓库根目录的 MIT。
