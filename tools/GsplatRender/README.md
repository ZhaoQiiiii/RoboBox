# GsplatRender

基于 3D Gaussian Splatting 的相机仿真。默认链路把渲染与 ROS 解耦成两个进程：纯 TCP
renderer 不依赖 ROS，ROS bridge 负责恢复原有话题；旧的 ROS 单进程入口保留，可随时回退。

## 目录结构

```
config/renderer_params.yaml       # 渲染分辨率、发布频率、相机内参
renderer/gsplat_renderer_ros1.py  # 旧 ROS 1 单进程节点（保持兼容）
renderer/tcp_protocol.py          # TCP 消息封装与校验（无 ROS/GPU 依赖）
renderer/gsplat_core.py           # 非 ROS 渲染核心
renderer/gsplat_renderer_tcp.py   # TCP renderer 服务端
renderer/gsplat_ros_bridge.py     # TCP → ROS 1 bridge
scripts/run.sh                    # 旧 ROS 入口
scripts/run_tcp.sh                # TCP renderer 入口
scripts/run_bridge.sh             # ROS bridge 入口
tests/                            # 协议、渲染核心、运行时与 bridge 测试
```

## 依赖

- TCP renderer：Python ≥ 3.8，`requirements.txt` 中的 torch/gsplat/numpy/scipy/plyfile/PyYAML；不需要 ROS。
- ROS bridge：ROS 1（`rospy`、`cv_bridge`、`sensor_msgs`、`nav_msgs`、`std_msgs`）；不依赖 torch/gsplat。
- 已验证环境：ROS Noetic、torch `2.7.0+cu128`、gsplat `1.5.3`、numpy `1.26.4`、scipy `1.15.3`。

## 运行

主链路默认使用全 TCP：

```bash
bash scripts/main.sh                         # 默认 GSPLAT_TRANSPORT=tcp
GSPLAT_TRANSPORT=ros1 bash scripts/main.sh   # 回退旧 ROS 单进程入口
```

TCP 模式下 `scripts/main.sh` 启动两个独立进程，日志分别为 `gsplat_renderer_tcp.log`
和 `gsplat_bridge.log`。也可以手工分别启动：

```bash
# 终端 1：TCP renderer，默认监听 127.0.0.1:19080（位姿）与 19081（图像）
bash runtime/RoboBox/tools/GsplatRender/scripts/run_tcp.sh /path/to/scene.ply --enable_gimbal

# 终端 2：ROS bridge，连接上述端口并订阅 /super/sim/lidar_slam/odom
bash runtime/RoboBox/tools/GsplatRender/scripts/run_bridge.sh
```

| 参数/环境变量 | 默认值 | 说明 |
|---|---|---|
| `GSPLAT_TRANSPORT` | `tcp` | `tcp` 双进程或 `ros1` 旧单进程 |
| `GSPLAT_POSE_PORT` | 19080 | 位姿连接端口（bridge → renderer） |
| `GSPLAT_FRAME_PORT` | 19081 | 图像连接端口（renderer → bridge） |
| `--host` | `127.0.0.1` | 首版仅同机部署 |
| `--ply_file` | `3dgs.ply` | 3DGS 场景 PLY |
| `--params_file` | `config/renderer_params.yaml` | 渲染参数与内参 |
| `--enable_gimbal` | 关闭 | 只使用 yaw，忽略 roll/pitch |
| `--odom-topic` | `/super/sim/lidar_slam/odom` | bridge 订阅的位姿话题 |

连接行为：bridge 对位姿和图像连接分别自动重连，间隔 1 秒；启动顺序任意。renderer
重启后 bridge 重新获取内参，不补发断开期间的旧帧；每种连接只接受一个客户端。TCP
传输原始 RGB/depth，不做有损压缩；640×480、15 Hz 时有效载荷约 23.04 MB/s。

旧 ROS 入口保持原行为，也可单独运行：

```bash
./scripts/run.sh /path/to/scene.ply --enable_gimbal
```

两个模式使用同名 topics，不能同时运行，否则会向 `/gsplat_render/*` 重复发布。

## 订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `/super/sim/lidar_slam/odom` | `nav_msgs/Odometry` | bridge 订阅；renderer 通过 TCP 接收位姿 |

## 发布

每帧只发布 `camera0`（前视）图像；`CameraInfo` 对 4 个相机位都 latched 发布。

| 话题 | 类型 | 说明 |
|---|---|---|
| `/gsplat_render/camera0/color/image` | `sensor_msgs/Image` | RGB，`rgb8` |
| `/gsplat_render/camera0/color/image/compressed` | `sensor_msgs/CompressedImage` | JPEG |
| `/gsplat_render/camera0/depth/image` | `sensor_msgs/Image` | 深度，`16UC1`（mm），裁剪到 100–20000 mm，alpha 掩码外的像素置 0 |
| `/gsplat_render/camera0/depth/image/compressed` | `sensor_msgs/CompressedImage` | PNG + 12 字节 ConfigHeader |
| `/gsplat_render/camera{0-3}/color/info` | `sensor_msgs/CameraInfo` | 彩色相机内参，latched |
| `/gsplat_render/camera{0-3}/depth/info` | `sensor_msgs/CameraInfo` | 深度相机内参，latched |

所有图像使用 odometry 的源时间戳与 `frame_id="camera"`。

## 测试

```bash
# 协议与运行时：普通 Python 环境即可
python3 -m unittest discover -s runtime/RoboBox/tools/GsplatRender/tests -p 'test_tcp_*.py' -v

# 渲染核心：需要 torch/gsplat
python3 -m unittest discover -s runtime/RoboBox/tools/GsplatRender/tests -p test_gsplat_core.py -v

# bridge：需要已配置 ROS 1 的环境
python3 -m unittest discover -s runtime/RoboBox/tools/GsplatRender/tests -p test_ros_bridge.py -v
```

## 已知限制

- 代码定义 4 个相机位（前/后/左/右），但只渲染并发布 `camera0`；其余相机位只有 latched `CameraInfo`。
- 首版仅同机、单 bridge，默认绑定 `127.0.0.1`；不引入 UDP、共享内存或消息队列。
- 灰度、反深度和点云 debug topic 原实现只有 publisher、没有实际发布，新链路不再创建这些空 topic。
- PLY 需包含 3DGS 属性：`x`/`y`/`z`、`f_dc_0..2`、`opacity`、`rot_0..3`、`scale_0..2`。
- 只支持 pinhole 相机模型。

## License

MIT，见 [LICENSE](LICENSE)。
