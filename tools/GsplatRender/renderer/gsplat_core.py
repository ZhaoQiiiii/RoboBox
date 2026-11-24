"""非 ROS 的 3DGS 渲染核心。

计算逻辑复制自同目录 ``gsplat_renderer_ros1.py``（该文件保持字节级不变）：
PLY 加载、camera0 view matrix、``RGB+ED`` rasterization 以及 RGB/depth 编码。
"""

from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
import torch
import yaml
from gsplat import rasterization
from plyfile import PlyData
from scipy.spatial.transform import Rotation

from tcp_protocol import DEPTH_ENCODING, FRAME_ID, RGB_ENCODING

CAMERA_NAMES = ["camera0", "camera1", "camera2", "camera3"]
MIN_DEPTH_MM = 100
MAX_DEPTH_MM = 20000
ALPHA_THRESHOLD = 0.5


class GsplatCore:
    def __init__(self, ply_file: Path, params_file: Path, enable_gimbal: bool = False) -> None:
        self.enable_gimbal = enable_gimbal
        self.device = str(torch.device("cuda" if torch.cuda.is_available() else "cpu"))

        # 渲染参数默认值与原实现一致，随后被 renderer_params.yaml 覆盖。
        self.width = 640
        self.height = 480
        self.publish_rate = 15.0
        self.cx = 320.0
        self.cy = 240.0
        self.fx = 618.6
        self.fy = 618.6

        self.load_camera_params(Path(params_file))
        self.means, self.quats, self.scales, self.colors, self.opacities = \
            self.load_ply_to_gsplat(Path(ply_file))

        self.intrin_mat = torch.tensor(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            device=self.device, dtype=torch.float32).unsqueeze(0)  # [1, 3, 3] for gsplat
        # camera0 外参为单位阵（原实现 extrin_mats[0]）。
        self.extrin_mat = np.eye(4, dtype=np.float32)

    def camera_metadata(self) -> Dict[str, Any]:
        """bridge 发布 CameraInfo 与 frame 消息所需的相机元数据。"""
        return {
            "width": int(self.width),
            "height": int(self.height),
            "fx": float(self.fx),
            "fy": float(self.fy),
            "cx": float(self.cx),
            "cy": float(self.cy),
            "frame_id": FRAME_ID,
            "cam_names": list(CAMERA_NAMES),
            "rgb_encoding": RGB_ENCODING,
            "depth_encoding": DEPTH_ENCODING,
        }

    def load_camera_params(self, params_file: Path) -> None:
        if not params_file.exists():
            raise FileNotFoundError("参数文件不存在: %s" % params_file)
        if params_file.suffix != ".yaml":
            raise ValueError("参数文件格式不正确: %s" % params_file)

        with open(params_file, "r") as params_stream:
            params = yaml.safe_load(params_stream)

        render_params = params.get("render", {})
        self.width = render_params.get("width", self.width)
        self.height = render_params.get("height", self.height)
        self.publish_rate = render_params.get("publish_rate", self.publish_rate)
        self.cx = render_params.get("cx", self.cx)
        self.cy = render_params.get("cy", self.cy)
        self.fx = render_params.get("fx", self.fx)
        self.fy = render_params.get("fy", self.fy)

        print("成功读取参数文件: %s" % params_file)
        print("size=%sx%s rate=%sHz fx=%s fy=%s cx=%s cy=%s" % (
            self.width, self.height, self.publish_rate, self.fx, self.fy, self.cx, self.cy))

    def load_ply_to_gsplat(self, ply_path: Path):
        print("加载 PLY 文件: %s" % ply_path)
        print("使用设备: %s" % self.device)
        plydata = PlyData.read(str(ply_path))
        vertices = plydata["vertex"]
        vertices_data = vertices.data
        vertex_names = list(vertices_data.dtype.names)

        if not all(coord in vertex_names for coord in ["x", "y", "z"]):
            raise ValueError("错误: PLY 文件中缺少 x, y, z 坐标！")
        means = torch.stack([
            torch.from_numpy(vertices_data["x"].astype(np.float32)),
            torch.from_numpy(vertices_data["y"].astype(np.float32)),
            torch.from_numpy(vertices_data["z"].astype(np.float32))], dim=1).to(self.device)

        if all("f_dc_%d" % index in vertex_names for index in range(3)):
            colors = torch.stack([
                torch.from_numpy(vertices_data["f_dc_%d" % index].astype(np.float32))
                for index in range(3)], dim=1).to(self.device)
            colors = torch.sigmoid(colors)
            colors = colors.unsqueeze(0)
        else:
            print("警告: 未找到颜色属性，使用默认颜色！")
            colors = torch.ones((len(vertices_data), 3), dtype=torch.float32,
                                device=self.device) * 0.8

        if "opacity" in vertex_names:
            opacities = torch.from_numpy(vertices_data["opacity"].astype(np.float32)).to(self.device)
            opacities = torch.sigmoid(opacities)
            opacities = opacities.squeeze(-1) if opacities.dim() == 2 else opacities
        else:
            print("警告: 未找到不透明度属性，使用默认值！")
            opacities = torch.ones(len(vertices_data), dtype=torch.float32,
                                   device=self.device) * 0.8

        if all("rot_%d" % index in vertex_names for index in range(4)):
            rotations_np = np.column_stack([
                vertices_data["rot_%d" % index] for index in range(4)]).astype(np.float32)
            norms = np.linalg.norm(rotations_np, axis=1, keepdims=True)
            rotations_np = rotations_np / norms
            quats = torch.from_numpy(rotations_np).to(self.device)
        else:
            print("警告: 未找到旋转属性，使用默认旋转！")
            quats = torch.zeros((len(vertices_data), 4), dtype=torch.float32, device=self.device)
            quats[:, 0] = 1.0

        if all("scale_%d" % index in vertex_names for index in range(3)):
            scales_np = np.column_stack([
                vertices_data["scale_%d" % index] for index in range(3)]).astype(np.float32)
            scales_np = np.exp(scales_np)
            scales = torch.from_numpy(scales_np).to(self.device)
        else:
            print("警告: 未找到尺度属性，使用默认尺度！")
            scales = torch.ones((len(vertices_data), 3), dtype=torch.float32,
                                device=self.device) * 0.01

        print("参数检查: means=%s colors=%s opacities=%s quats=%s scales=%s" % (
            tuple(means.shape), tuple(colors.shape), tuple(opacities.shape),
            tuple(quats.shape), tuple(scales.shape)))
        return means, quats, scales, colors, opacities

    def camera0_view_matrix(self, position: np.ndarray, orientation: np.ndarray) -> np.ndarray:
        """world frame → camera0 的 view matrix，公式与原实现 create_camera_viewmat(cam_id=0) 相同。"""
        position = np.asarray(position, dtype=np.float32)
        orientation = np.asarray(orientation, dtype=np.float32)

        if self.enable_gimbal:
            yaw = Rotation.from_quat(orientation).as_euler("zyx")[0]
            R_w2b = Rotation.from_euler("z", yaw).as_matrix().T
        else:
            R_w2b = Rotation.from_quat(orientation).as_matrix().T

        R_b2c = np.array([
            [0, -1, 0],
            [0, 0, -1],
            [1, 0, 0],
        ], dtype=np.float32)

        R = R_b2c @ R_w2b
        t = -R @ position

        viewmat = np.eye(4, dtype=np.float32)
        viewmat[:3, :3] = R
        viewmat[:3, 3] = t
        return self.extrin_mat @ viewmat

    def render(self, position: np.ndarray, orientation: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """渲染 camera0，返回连续的 (rgb_uint8 [H,W,3], depth_uint16 [H,W])。"""
        viewmat = torch.tensor(
            self.camera0_view_matrix(position, orientation), device=self.device).unsqueeze(0)

        render_colors, render_alphas, _ = rasterization(
            means=self.means,
            quats=self.quats,
            scales=self.scales,
            opacities=self.opacities,
            colors=self.colors,
            viewmats=viewmat,
            Ks=self.intrin_mat,
            width=self.width,
            height=self.height,
            near_plane=0.01,
            far_plane=100.0,
            radius_clip=0.0,
            eps2d=0.1,
            tile_size=16,
            backgrounds=None,
            render_mode="RGB+ED",
            sparse_grad=False,
            absgrad=False,
            rasterize_mode="classic",
            camera_model="pinhole",
            packed=True,
            sh_degree=None,
            channel_chunk=32,
            distributed=False,
            segmented=False,
            with_ut=False,
            with_eval3d=False,
        )

        rgb = render_colors.squeeze(0)[..., :3]
        depth = render_colors.squeeze(0)[..., 3:4]
        alpha = render_alphas.squeeze(0)

        rgb_np = rgb.detach().cpu().numpy().astype(np.float32)
        depth_np = depth.detach().cpu().numpy().astype(np.float32)
        alpha_np = alpha.detach().cpu().numpy().astype(np.float32)
        return encode_render_arrays(rgb_np, depth_np, alpha_np)


def encode_render_arrays(rgb: np.ndarray, depth: np.ndarray,
                         alpha: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """按原实现的顺序把渲染浮点数组转成 RGB uint8 与 depth uint16（毫米）。"""
    rgb_array = np.asarray(rgb, dtype=np.float32)
    height, width = rgb_array.shape[:2]
    rgb_uint8 = (rgb_array * 255).astype(np.uint8)

    depth_array = np.asarray(depth, dtype=np.float32).reshape(height, width)
    depth_mm = depth_array.copy() * 1000.0
    depth_uint16 = depth_mm.astype(np.uint16)
    depth_uint16 = depth_uint16.clip(MIN_DEPTH_MM, MAX_DEPTH_MM)

    alpha_array = np.asarray(alpha, dtype=np.float32).reshape(height, width)
    depth_uint16[alpha_array < ALPHA_THRESHOLD] = 0
    # 线上载荷约定小端 uint16。
    return np.ascontiguousarray(rgb_uint8), np.ascontiguousarray(depth_uint16, dtype="<u2")
