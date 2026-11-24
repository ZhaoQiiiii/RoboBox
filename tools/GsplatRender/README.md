# GsplatRender

基于 3D Gaussian Splatting 的 ROS 1 相机仿真节点。加载 3DGS 场景 PLY，订阅里程计，按当前位姿用 [gsplat](https://github.com/nerfstudio-project/gsplat) 渲染 RGB 与深度图并发布为 ROS 话题，供感知、规划与可视化节点消费。

## 目录结构

```
config/renderer_params.yaml       # 渲染分辨率、发布频率、相机内参
renderer/gsplat_renderer_ros1.py  # ROS 1 节点（rospy）
scripts/run.sh                    # 启动入口
```

## 依赖

- ROS 1（提供 `rospy`、`cv_bridge`、`sensor_msgs`、`nav_msgs`、`std_msgs`）
- Python ≥ 3.8
- pip 依赖见 `requirements.txt`；torch 需安装与本机 CUDA 驱动匹配的版本
- 已验证环境：ROS Noetic、torch `2.7.0+cu128`、gsplat `1.5.3`、numpy `1.26.4`、scipy `1.15.3`

## 运行

```bash
./scripts/run.sh /path/to/scene.ply                 # 完整姿态渲染
./scripts/run.sh /path/to/scene.ply --enable_gimbal # 云台模式，仅使用 yaw
```

直接运行节点：

```bash
python3 renderer/gsplat_renderer_ros1.py --ply_file /path/to/scene.ply
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--ply_file` | `3dgs.ply` | 3DGS 场景 PLY |
| `--params_file` | `config/renderer_params.yaml` | 渲染参数与内参，默认值按脚本位置解析 |
| `--enable_gimbal` | 关闭 | 只使用 yaw，忽略 roll/pitch |

## 订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `/super/sim/lidar_slam/odom` | `nav_msgs/Odometry` | 渲染位姿；当前为固定话题名，复用时需修改 `renderer/gsplat_renderer_ros1.py` |

## 发布

每帧只发布 `camera0`（前视）图像；`CameraInfo` 对 4 个相机位都 latched 发布。

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gsplat_render/camera0/color/image` | `sensor_msgs/Image` | RGB，`rgb8` |
| `/gsplat_render/camera0/color/image/compressed` | `sensor_msgs/CompressedImage` | JPEG |
| `/gsplat_render/camera0/depth/image` | `sensor_msgs/Image` | 深度，`16UC1`（mm），裁剪到 100–20000 mm，alpha 掩码外的像素置 0 |
| `/gsplat_render/camera0/depth/image/compressed` | `sensor_msgs/CompressedImage` | PNG + 12 字节 ConfigHeader |
| `/gsplat_render/camera0/color/info` | `sensor_msgs/CameraInfo` | 彩色相机内参，latched |
| `/gsplat_render/camera0/depth/info` | `sensor_msgs/CameraInfo` | 深度相机内参，latched |

## 已知限制

- 代码定义 4 个相机位（前/后/左/右），但只发布 `camera0`。
- 订阅话题 `/super/sim/lidar_slam/odom` 与所有 `frame_id`（`camera`）为固定值。
- PLY 需包含 3DGS 属性：`x`/`y`/`z`、`f_dc_0..2`、`opacity`、`rot_0..3`、`scale_0..2`。
- 只支持 pinhole 相机模型。

## License

MIT，见 [LICENSE](LICENSE)。
