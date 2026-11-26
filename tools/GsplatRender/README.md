# GsplatRender

基于 3D Gaussian Splatting 的相机仿真。TCP renderer 负责 GPU 渲染，ROS bridge 把
RGB/depth 帧转成 ROS 1 话题；两者通过 TCP 解耦，也可以放在同一个容器里运行。

## 目录结构

```
config/params.yaml             # 渲染分辨率、发布频率、相机内参
runtime/tcp_protocol.py        # TCP 消息封装与校验（无 ROS/GPU 依赖）
runtime/gsplat_core.py         # 非 ROS 渲染核心
runtime/gsplat_render_tcp.py   # TCP renderer 服务端
runtime/gsplat_ros_bridge.py   # TCP → ROS 1 bridge
scripts/build.sh               # 构建容器镜像（只需一次）
scripts/run.sh                 # 运行容器（renderer + bridge）
scripts/stop.sh                # 停止并删除容器
docker/                        # 容器 Dockerfile 与 entrypoint
```

## 依赖

- TCP renderer：Python ≥ 3.8，`requirements.txt` 中的 torch/gsplat/numpy/scipy/plyfile/PyYAML；不需要 ROS。
- ROS bridge：ROS 1（`rospy`、`cv_bridge`、`sensor_msgs`、`nav_msgs`）；不依赖 torch/gsplat。
- 已验证环境：ROS Noetic、torch `2.7.0+cu128`、gsplat `1.5.3`、numpy `1.26.4`、scipy `1.15.3`。

## 运行

推荐用容器。镜像只提供依赖，`scripts/run.sh` 始终把宿主 `runtime/`、`config/` 和
`docker/entrypoint.sh` 挂进容器，所以镜像只需要构建一次：

```bash
bash scripts/build.sh
bash scripts/run.sh --detach /path/to/scene.ply
docker logs -f gsplat-render
bash scripts/stop.sh
```

容器内由 `docker/entrypoint.sh` 同时启动 renderer 和 bridge，云台模式默认开启；
bridge 需要外部可达的 ROS master。修改 `runtime/`、`config/` 或 `docker/entrypoint.sh` 后：

```bash
docker restart gsplat-render
```

不需要重建镜像；只有改 `docker/Dockerfile` 或依赖时才重新执行 `scripts/build.sh`。

已安装依赖时也可以手工启动两个进程：

```bash
# 终端 1：TCP renderer，默认监听 127.0.0.1:19080（位姿）与 19081（图像）
python3 runtime/gsplat_render_tcp.py --ply_file /path/to/scene.ply --enable_gimbal

# 终端 2：ROS bridge，连接上述端口并订阅 /super/sim/lidar_slam/odom
source /opt/ros/noetic/setup.bash
python3 runtime/gsplat_ros_bridge.py
```

| 参数/环境变量 | 默认值 | 说明 |
|---|---|---|
| `GSPLAT_POSE_PORT` / `GSPLAT_FRAME_PORT` | 19080 / 19081 | 容器 entrypoint 的端口，分别传给 `--pose-port` / `--frame-port` |
| `--host` | `127.0.0.1` | renderer 绑定地址，也是 bridge 连接地址 |
| `--ply_file` | `3dgs.ply` | 3DGS 场景 PLY |
| `--params_file` | `config/params.yaml` | 渲染参数与内参 |
| `--enable_gimbal` | 关闭（容器内固定开启） | 只使用 yaw，忽略 roll/pitch |
| `--odom-topic` | `/super/sim/lidar_slam/odom` | bridge 订阅的位姿话题 |

连接行为：bridge 对位姿和图像连接分别自动重连，间隔 1 秒；启动顺序任意。renderer
重启后 bridge 重新获取内参，不补发断开期间的旧帧；每种连接只接受一个客户端。TCP
传输原始 RGB/depth，不做有损压缩；640×480、15 Hz 时有效载荷约 23.04 MB/s。

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

## 日志

- renderer：连接建立/断开时打印；运行中每 5 秒一行
  `[INFO] [render] 640 x 480, 15.0 fps, seq=<累计图像数>`。
- bridge：静默运行，只在连接建立时打印
  `[INFO] <时间戳>: bridge 位姿连接已建立` / `bridge 图像连接已建立`。

## 已知限制

- 代码定义 4 个相机位（前/后/左/右），但只渲染并发布 `camera0`；其余相机位只有 latched `CameraInfo`。
- 首版仅同机、单 bridge，默认绑定 `127.0.0.1`；不引入 UDP、共享内存或消息队列。
- 灰度、反深度和点云 debug topic 原实现只有 publisher、没有实际发布，新链路不再创建这些空 topic。
- PLY 需包含 3DGS 属性：`x`/`y`/`z`、`f_dc_0..2`、`opacity`、`rot_0..3`、`scale_0..2`。
- 只支持 pinhole 相机模型。

## License

MIT，见 [LICENSE](LICENSE)。
