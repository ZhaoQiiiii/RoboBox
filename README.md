# RoboKit

RoboKit 是本地机器人仿真工具集：`tools/3D-Toolbox` 是三维场景 Web 编辑器；
`tools/GsplatRender` 和 `tools/LidarRender` 是两个传感器渲染工具，共用
`robokit:latest` 镜像和同一个 `robokit` 容器。

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)

## 仓库组成

| 路径 | 说明 |
|---|---|
| `tools/3D-Toolbox/` | 三维 Web 编辑器（React + Vite + 内置后端 API） |
| `tools/GsplatRender/` | 3DGS 相机仿真：PLY → RGB/深度图（TCP + ROS bridge） |
| `tools/LidarRender/` | 激光雷达仿真：PCD/点云 PLY → 扫描点云（TCP + ROS bridge） |
| `tools/RerunBridge/` | 可选：把 Gsplat/Lidar 的 ROS 图像与点云转发到 Rerun |
| `scripts/build.sh` | 构建统一镜像 `robokit:latest` |
| `scripts/run.sh` | 启动统一容器 `robokit`（`gsplat` / `lidar` / `both`） |
| `docker/` | Dockerfile 与 dispatcher entrypoint |
| `tools/3D-Toolbox/scenes/` | Web 编辑器场景数据桶（gitignore，仅保留 `.gitkeep`） |
| `logs/` | 运行日志（gitignore） |

## 快速开始

### 传感器工具（Docker）

```bash
bash scripts/build.sh
bash scripts/run.sh --ply /path/to/scene.ply --pcd /path/to/scene.pcd
```

- `--ply`：3DGS 场景，给 GsplatRender；
- `--pcd`：激光地图，给 LidarRender，接受 `.pcd` 或点云 `.ply`（启动时转成 binary PCD）；
- 至少给一个；只给一个时只启动对应工具；两个都给时在同一容器内并行运行；
- 默认容器名 `robokit`，前台运行；加 `-d` 后台运行，`docker logs -f robokit` 查看日志；
- 需要外部 ROS master 与 odom 源；ZeroSim 主链路由仓库根目录的 `scripts/run.sh` 负责启动它们并调用本目录的脚本。

### Rerun 可视化（可选）

`tools/RerunBridge` 在同一个容器里额外启动一个 Python 3.10 的 Rerun sink 和一个
Python 3.8 的 ROS tee，把渲染结果实时转发到 Rerun：

```bash
bash scripts/run.sh --ply /path/to/scene.ply --pcd /path/to/scene.pcd --rerun
```

- 默认连接 `rerun+http://127.0.0.1:9877/proxy`，application id 为 `robokit`，在 viewer
  里是一条独立 recording（与 DiMOS 主 recording 分开，可在 recordings 列表切换）；
- 实体路径：`/robokit/gsplat/color`、`/robokit/gsplat/depth`、`/robokit/lidar/cloud`；
- 默认只转发彩色图（`/gsplat_render/camera0/color/image/compressed`，最高 15 Hz）和
  点云（`--rerun-cloud-topic`，默认取 `--lidar-local-topic`，5 Hz、最多 2 万点）；
  深度默认关闭，用 `--rerun-depth-topic` 打开并配合 `--rerun-image-topic` 调整；
- standalone 使用：先启动自己的 Rerun viewer，再给 `--rerun` 或 `--rerun-url`；
  需要 RoboKit 自己起 gRPC server 时用 `--rerun-serve --rerun-port 9880`，然后用
  `rerun rerun+http://127.0.0.1:9880/proxy` 连接；
- 布局修改：viewer 里调好的布局由 `--persist-state` 跨运行保留；要固定或共享时用
  **Save blueprint** 导出 `.rbl`，并通过 `DIMOS_VIEWER_BLUEPRINTS`（`:` 分隔）在
  `dimos-viewer` 启动时加载。sink 发送 blueprint 时不会覆盖已保存的布局；
- 版本约定：镜像内固定 `rerun-sdk==0.32.0`（独立 venv，不影响 gsplat 的 numpy 1.x），
  viewer/server 需要用同版本；
- bridge 需要 ROS master 已启动；tee 会在 master 不可达时每 2 秒重试，sink 在 Rerun
  server 不可达时也会持续重试。`--rerun` 默认关闭，不影响四旋翼链路。

### 三维编辑器

```bash
cd tools/3D-Toolbox
bun install
bun run dev      # http://localhost:5173
```

| 命令 | 说明 |
|---|---|
| `bun run dev` | 启动 Vite 开发服务器与内置后端 API（端口占用时自动回退） |
| `bun run build` | 类型检查并构建生产产物到 `frontend/dist/` |
| `bun run preview` | 预览已构建的产物 |
| `bunx tsc -p frontend --noEmit` | 仅类型检查 |
| `bun test` | 运行 `bun:test` 测试 |

## 场景数据

Web 编辑器后端固定读取 **`tools/3D-Toolbox/scenes/`**：

```ts
const APP_ROOT = join(import.meta.dirname, "..");
const SCENES_ROOT = join(APP_ROOT, "scenes");
```

```text
tools/3D-Toolbox/scenes/<scene>/
  <asset>.pcd / <asset>.ply / <asset>.splat   # 点云/高斯资产（扁平存放）
  bboxes/<asset>.json                          # 包围框标注
  scene_graph_saved/<snapshot>/                # 场景图快照（只读来源）
  scene_graph_exported/<snapshot>/             # 场景图导出结果
  trajectories/                                # 轨迹（*.json 或 flight_* / step_*）
```

`tools/3D-Toolbox/scenes/` 默认被 `.gitignore` 忽略，只保留 `.gitkeep`。

## 传感器工具

### GsplatRender

- 输入：3DGS PLY（高斯泼溅）；
- TCP：`19080`（位姿）、`19081`（图像）；
- ROS bridge：发布 `/gsplat_render/camera0/color/image`、`/gsplat_render/camera0/depth/image`
  等，订阅 odom（默认 `/super/sim/lidar_slam/odom`）；
- 单独运行：`bash tools/GsplatRender/scripts/run.sh /path/scene.ply`。

### LidarRender

- 输入：PCD 或点云 PLY（`x y z` 必需，`intensity` 可选）；PLY 在容器启动时由
  `runtime/ply_to_pcd.py` 转成 binary PCD；
- TCP：`19082`（位姿）、`19083`（点云）；
- ROS bridge：发布 `/super/sim/cloud_registered`（局部扫描）与 `/super/sim/global_pc`
  （整图，首个订阅者出现时发布一次），订阅 odom；
- 默认参数：`lidar_type=3`（MID_360）、`sensing_rate=10`、`sensing_horizon=10`；
- 单独运行：`bash tools/LidarRender/scripts/run.sh /path/scene.pcd`；
- 3DGS PLY 也能转换，但只取高斯中心点、`intensity=0`，不是真正的表面地图；优先使用
  专用 PCD 或点云 PLY。

### 统一容器

`scripts/run.sh` 通过 dispatcher 选择 `gsplat` / `lidar` / `both`；`both` 会在同一容器内
并行启动两个工具，日志分别写到 `/logs/gsplat_renderer.log`、`/logs/lidar_render.log`。
宿主 `tools/*/runtime`、`tools/*/config` 与 `docker/entrypoint.sh` 会挂载进容器；修改这些
文件后重启容器即可生效，不需要重新构建镜像。

## 镜像构建

`bash scripts/build.sh` 等价于 `docker build -f docker/Dockerfile -t robokit:latest .`，
并处理构建代理与 Ubuntu/ROS 镜像源。Dockerfile 分四个阶段：

| 阶段 | 基础镜像 | 作用 |
|---|---|---|
| `gsplat-builder` | CUDA 12.8.1 devel（Ubuntu 20.04） | 编译 gsplat 并生成 `/opt/gsplat-venv` |
| `lidar-builder` | Ubuntu 20.04 | 编译 C++ `lidar_render_tcp` |
| `robokit-runtime-base` | CUDA 12.8.1 runtime（Ubuntu 20.04） | ROS Noetic + PCL/OpenCV/GL 等运行依赖 |
| `robokit` | 上述 runtime base | 复制两个工具的产物与 dispatcher，生成最终镜像 |

## 三维编辑器功能

- **三种懒加载模式**：`3D-BBox Annotator`（包围框标注）、`SceneGraph Editor`（场景图编辑）、
  `3D-Trajectory Viewer`（轨迹查看），通过右上角按钮或 URL hash `#mode=bbox|scenegraph|traj`
  切换。
- **持久 WebGL Canvas**：三个模式共用一个画布，切换模式不重新加载场景，保留高斯泼溅 GPU
  资源与点云缓冲。
- **点云 / 高斯泼溅渲染**：基于 `@mkkellogg/gaussian-splats-3d`，支持 `.pcd` / `.ply` /
  `.splat` 等资产格式。
- **点云按高度着色**：Flat / Rainbow / Cool-Warm / Grayscale / X-Ray / Y-Ray 等配色方案。
- **共享可视化参数**：点大小、栅格尺度、坐标系显示、配色与不透明度跨模式一致、跨会话持久化。
- **场景图编辑**：节点、区域、多面体与对象的增删改，撤销/重做、连接等快捷键，可导出为新的
  场景图快照。
- **轨迹可视化**：只读浏览 `scenes/<scene>/trajectories/` 下的 `*.json` 与 worldmodel
  `flight_*` / `step_*` 目录。
- **内置后端 API**：Vite 开发服务器的插件直接提供场景 / 包围框 / 场景图 / 轨迹相关 REST 接口。

## 项目结构

```text
tools/3D-Toolbox/
  backend/            # Vite 插件形式的 REST API
    api-plugin.ts
  frontend/           # React + Vite（Vite root）
    src/
      App.tsx         # 持久 WebGL <Canvas> 与模式切换
      modes/          # bbox / scenegraph / traj
      shared/         # 跨模式组件、hook 与工具
  scenes/             # 场景数据桶
  package.json
  bun.lock

tools/GsplatRender/   # 相机渲染 + ROS bridge
tools/LidarRender/    # 雷达渲染 + ROS bridge
scripts/              # build.sh / run.sh
docker/               # Dockerfile / entrypoint.sh
logs/                 # 运行日志
```

## 测试与验证

```bash
# 编辑器
cd tools/3D-Toolbox
bunx tsc -p frontend --noEmit
bun test

# LidarRender（容器内，需要 numpy / rospy）
cd tools/LidarRender
PYTHONPATH=runtime /usr/bin/python3 -m unittest discover -s tests -p 'test_tcp_protocol.py'
PYTHONPATH=runtime /usr/bin/python3 -m unittest discover -s tests -p 'test_ply_to_pcd.py'

# Dockerfile 静态检查
docker build --check -f docker/Dockerfile .
```

## 许可

RoboKit 自有代码沿用根目录的 MIT。`tools/LidarRender/marsim_render/` 复制自 SUPER，
保留 LGPL-3.0；见 `tools/LidarRender/README-SOURCE.md`。
