#!/usr/bin/env python3

import argparse
import os
import struct
import threading
from pathlib import Path
from typing import Literal

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import torch
import yaml
from cv_bridge import CvBridge
from gsplat import rasterization
from nav_msgs.msg import Odometry
from plyfile import PlyData
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class GsplatRenderer:
    def __init__(self, ply_file, params_file, enable_gimbal=False):
        self.enable_rgb_pub = True
        self.enable_depth_pub = True
        self.enable_gimbal = enable_gimbal  # 云台模式：仅使用 yaw
        rospy.loginfo(f"云台模式: {'开启 (仅 yaw 渲染)' if self.enable_gimbal else '关闭 (完整姿态渲染)'}")

        self.lock = threading.Lock()
        self.device = str(torch.device("cuda" if torch.cuda.is_available() else "cpu"))
        self.have_odom = False
        self.new_odom_received = False
        self.position = None
        self.orientation = None
        self.stamp_odom = None
        self.viewmat = None

        self.depth_stats = None
        self.show_depth_stats = True

        # Gaussian Render Params
        # 注意：此默认值会被 renderer_params.yaml 的 render 段覆盖，保持与其一致
        self.width = 640
        self.height = 480
        self.publish_rate = 15.0
        self.cx = 320.0
        self.cy = 240.0
        self.fx = 618.6
        self.fy = 618.6

        # Depth Process Params
        self.min_depth = 100.0  # mm (改为 100，更灵敏)
        self.max_depth = 20000.0  # mm
        self.filter = True
        self.fill_holes = True
        self.remove_outliers = True
        self.alpha_threshold = 0.5  # alpha 掩码阈值

        # Load render params & 3DGS model
        self.load_camera_params(params_file)
        self.means, self.quats, self.scales, self.colors, self.opacities = self.load_ply_to_gsplat(ply_file)

        # Extrinsic & Intrinsic matrix
        self.cam_names = ['camera0', 'camera1', 'camera2', 'camera3']  # 0:front, 1:back, 2:left, 3:right
        rotations = [
            np.eye(3, dtype=np.float32),
            Rotation.from_euler('y', 180, degrees=True).as_matrix(),
            Rotation.from_euler('y', 90, degrees=True).as_matrix(),
            Rotation.from_euler('y', -90, degrees=True).as_matrix()
        ]

        self.extrin_mats = []
        for cam_id in range(len(self.cam_names)):
            extrin_mat = np.eye(4, dtype=np.float32)
            extrin_mat[:3, 3] = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # translation
            extrin_mat[:3, :3] = rotations[cam_id]  # rotation
            self.extrin_mats.append(extrin_mat)  # [4, 4]

        self.intrin_mat = torch.tensor([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
                                       device=self.device, dtype=torch.float32).unsqueeze(0)  # [1, 3, 3] for gsplat

        # ROS Utils
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.imagePubCallback)
        rospy.Subscriber("/super/sim/lidar_slam/odom", Odometry, self.odomCallback)

        self.rgb_raw_pubs, self.rgb_gray_pubs = [], []
        self.depth_raw_pubs, self.depth_inv_pubs, self.depth_debug_pubs = [], [], []
        self.rgb_compressed_pubs, self.depth_compressed_pubs = [], []
        for cam_name in self.cam_names:
            self.rgb_raw_pubs.append(rospy.Publisher(f"/gsplat_render/{cam_name}/color/image", ImageMsg, queue_size=10))
            self.rgb_gray_pubs.append(rospy.Publisher(f"/gsplat_render/{cam_name}/color/image/gray", ImageMsg, queue_size=10))
            self.depth_raw_pubs.append(rospy.Publisher(f"/gsplat_render/{cam_name}/depth/image", ImageMsg, queue_size=10))
            self.depth_inv_pubs.append(rospy.Publisher(f"/gsplat_render/{cam_name}/depth/image/inv", ImageMsg, queue_size=10))
            self.depth_debug_pubs.append(rospy.Publisher(f"/gsplat_render/{cam_name}/depth/image/point", PointCloud2, queue_size=10))
            self.rgb_compressed_pubs.append(rospy.Publisher(f"/gsplat_render/{cam_name}/color/image/compressed", CompressedImage, queue_size=10))
            self.depth_compressed_pubs.append(rospy.Publisher(f"/gsplat_render/{cam_name}/depth/image/compressed", CompressedImage, queue_size=10))

        # cv_bridge：raw -> compressed 压缩发布
        self.cv_bridge = CvBridge()

        # CameraInfo latched 发布：每个相机各一份，供下游节点动态对齐内参。
        self.color_info_pubs = {}
        self.depth_info_pubs = {}
        for cam_name in self.cam_names:
            self.color_info_pubs[cam_name] = rospy.Publisher(
                f"/gsplat_render/{cam_name}/color/info", CameraInfo, queue_size=1, latch=True)
            self.depth_info_pubs[cam_name] = rospy.Publisher(
                f"/gsplat_render/{cam_name}/depth/info", CameraInfo, queue_size=1, latch=True)
        self.publish_camera_info()

    # Basic
    def publish_camera_info(self):
        """将渲染内参以 CameraInfo 发布（latched，仅需一次），供下游节点动态对齐内参。"""
        info = CameraInfo()
        info.header.frame_id = "camera"
        info.height = self.height
        info.width = self.width
        info.distortion_model = "plumb_bob"
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [self.fx, 0.0, self.cx,
                  0.0, self.fy, self.cy,
                  0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.P = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        for pubs in (self.color_info_pubs, self.depth_info_pubs):
            for pub in pubs.values():
                pub.publish(info)
        rospy.loginfo(f"CameraInfo 已发布(latched): {len(self.color_info_pubs)} 个相机, {self.width}x{self.height}, fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def load_camera_params(self, params_file):
        if not os.path.exists(params_file):
            rospy.logerr(f"参数文件不存在: {params_file}")
            exit(1)

        if params_file.suffix != '.yaml':
            rospy.logerr(f"参数文件格式不正确: {params_file}")
            exit(1)

        try:
            with open(params_file, 'r') as f:
                params = yaml.safe_load(f)

            if 'render' in params:
                render_params = params['render']
                self.width = render_params.get('width')
                self.height = render_params.get('height')
                self.publish_rate = render_params.get('publish_rate')
                self.cx = render_params.get('cx')
                self.cy = render_params.get('cy')
                self.fx = render_params.get('fx')
                self.fy = render_params.get('fy')

            rospy.loginfo(f"成功读取参数文件: {params_file}")
            rospy.loginfo(f"参数配置:")
            rospy.loginfo(f"size={self.width}x{self.height}")
            rospy.loginfo(f"rate={self.publish_rate}Hz")
            rospy.loginfo(f"cx={self.cx}, cy={self.cy}")
            rospy.loginfo(f"fx={self.fx}, fy={self.fy}")

        except Exception as e:
            rospy.logerr(f"文件解析错误: {e}")
            exit(1)

    def load_ply_to_gsplat(self, ply_path):
        print(f"加载 PLY 文件: {ply_path}")
        print(f"使用设备: {self.device}")
        plydata = PlyData.read(ply_path)
        vertices = plydata['vertex']
        vertices_data = vertices.data  # 直接访问 PlyElement 的内部 numpy 数组
        vertex_names = list(vertices_data.dtype.names)  # 获取属性名称
        # print(f"找到的属性 ({len(vertex_names)}个): {vertex_names}")

        # 1. means
        if not all(coord in vertex_names for coord in ['x', 'y', 'z']):
            raise ValueError("错误: PLY 文件中缺少 x, y, z 坐标！")
        means = torch.stack([
            torch.from_numpy(vertices_data['x'].astype(np.float32)),
            torch.from_numpy(vertices_data['y'].astype(np.float32)),
            torch.from_numpy(vertices_data['z'].astype(np.float32))], dim=1).to(self.device)  # [N, 3]

        # 2. colors
        if all(f'f_dc_{i}' in vertex_names for i in range(3)):  # 3DGS 格式
            colors = torch.stack([torch.from_numpy(vertices_data[f'f_dc_{i}'].astype(np.float32)) for i in range(3)], dim=1).to(self.device)
            colors = torch.sigmoid(colors)  # 必须使用 sigmoid 激活 --> [0,1]
            colors = colors.unsqueeze(0)  # [1, N, 3]
        else:
            print("警告: 未找到颜色属性，使用默认颜色！")
            colors = torch.ones((len(vertices_data), 3), dtype=torch.float32, device=self.device) * 0.8

        # 3. opacity
        if 'opacity' in vertex_names:
            opacities = torch.from_numpy(vertices_data['opacity'].astype(np.float32)).to(self.device)
            opacities = torch.sigmoid(opacities)
            opacities = opacities.squeeze(-1) if opacities.dim() == 2 else opacities  # [N]
        else:
            print("警告: 未找到不透明度属性，使用默认值！")
            opacities = torch.ones(len(vertices_data), dtype=torch.float32, device=self.device) * 0.8

        # 4. rotation (quats)
        if all(f'rot_{i}' in vertex_names for i in range(4)):
            rotations_np = np.column_stack([vertices_data[f'rot_{i}'] for i in range(4)]).astype(np.float32)
            norms = np.linalg.norm(rotations_np, axis=1, keepdims=True)
            rotations_np = rotations_np / norms
            quats = torch.from_numpy(rotations_np).to(self.device)  # [N, 4]
        else:
            print("警告: 未找到旋转属性，使用默认旋转！")
            quats = torch.zeros((len(vertices_data), 4), dtype=torch.float32, device=self.device)
            quats[:, 0] = 1.0

        # 5. scales
        if all(f'scale_{i}' in vertex_names for i in range(3)):
            scales_np = np.column_stack([vertices_data[f'scale_{i}'] for i in range(3)]).astype(np.float32)
            scales_np = np.exp(scales_np)
            scales = torch.from_numpy(scales_np).to(self.device)  # [N, 3]
        else:
            print("警告: 未找到尺度属性，使用默认尺度！")
            scales = torch.ones((len(vertices_data), 3), dtype=torch.float32, device=self.device) * 0.01

        print(f"\n参数检查:")
        print(f"  means: {means.shape}")
        print(f"  colors: {colors.shape}")
        print(f"  opacities: {opacities.shape}")
        print(f"  quats: {quats.shape}")
        print(f"  scales: {scales.shape}")
        return means, quats, scales, colors, opacities

    def create_camera_viewmat(self, position, orientation, cam_id=0):
        # 计算 view matrix: world frame → gsplat camera frame

        if self.enable_gimbal:
            yaw = Rotation.from_quat(orientation).as_euler('zyx')[0]
            R_w2b = Rotation.from_euler('z', yaw).as_matrix().T
        else:
            R_w2b = Rotation.from_quat(orientation).as_matrix().T  # [3, 3]

        R_b2c = np.array([
            [0, -1, 0],
            [0, 0, -1],
            [1, 0, 0]
        ], dtype=np.float32)

        R = R_b2c @ R_w2b
        t = -R @ position

        viewmat = np.eye(4, dtype=np.float32)
        viewmat[:3, :3] = R
        viewmat[:3, 3] = t
        viewmat = self.extrin_mats[cam_id] @ viewmat  # 相机外参矩阵
        viewmat = torch.tensor(viewmat, device=self.device).unsqueeze(0)  # [1, 4, 4] for gsplat
        return viewmat

    def render_with_gsplat(self, viewmats, Ks):
        """
        Input:
            means: [N, 3]
            quats: [N, 4]
            scales: [N, 3]
            opacities: [N]
            colors: [N, 3]
            viewmats: [1, 4, 4]
            Ks: [1, 3, 3]

        Output:
            rgb: [H, W, 3] or None
            alpha: [H, W, 1]
            depth: [H, W, 1] or None
        """

        # 1. 设置渲染模式
        # RGB: colored image
        # D: accumulated depth
        # ED: expected depth
        # render_mode: Literal["RGB", "D", "ED", "RGB+D", "RGB+ED"] = "RGB+D"
        render_mode: Literal["RGB", "D", "ED", "RGB+D", "RGB+ED"] = "RGB+ED"

        # 2. 使用 gsplat 渲染图像
        render_colors, render_alphas, meta = rasterization(
            means=self.means,
            quats=self.quats,
            scales=self.scales,
            opacities=self.opacities,
            colors=self.colors,
            viewmats=viewmats,
            Ks=Ks,
            width=self.width,
            height=self.height,
            near_plane=0.01,
            far_plane=100.0,
            radius_clip=0.0,
            eps2d=0.1,
            tile_size=16,
            backgrounds=None,
            render_mode=render_mode,
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

        # 3. 处理渲染结果
        rgb = depth = None
        if render_mode == "RGB":
            # render_colors: [1, H, W, 3]
            rgb = render_colors.squeeze(0)  # [H, W, 3]

        elif render_mode == "D" or render_mode == "ED":
            # render_colors: [1, H, W, 1]
            depth = render_colors.squeeze(0)  # [H, W, 1]

        elif render_mode == "RGB+D" or render_mode == "RGB+ED":
            # render_colors: [1, H, W, 4]
            rgb = render_colors.squeeze(0)[..., :3]  # [H, W, 3]
            depth = render_colors.squeeze(0)[..., 3:4]  # [H, W, 1]

        alpha = render_alphas.squeeze(0)  # [1, H, W, 1] -> [H, W, 1]

        # 4. 转成 numpy 数组
        rgb_np = rgb.detach().cpu().numpy().astype(np.float32) if rgb is not None else None
        alpha_np = alpha.detach().cpu().numpy().astype(np.float32) if alpha is not None else None
        depth_np = depth.detach().cpu().numpy().astype(np.float32) if depth is not None else None
        return rgb_np, alpha_np, depth_np

    # ROS
    def odomCallback(self, msg):
        with self.lock:
            if not self.have_odom: self.have_odom = True
            self.new_odom_received = True

            self.position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ], dtype=np.float32)

            self.orientation = np.array([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ], dtype=np.float32)

            self.stamp_odom = msg.header.stamp

    def imagePubCallback(self, _):
        if not self.have_odom:
            rospy.logwarn_throttle(1.0, "No odom received yet.")
            return

        with self.lock:
            if not self.new_odom_received: return
            self.new_odom_received = False
            position = self.position.copy()
            orientation = self.orientation.copy()
            stamp_odom = self.stamp_odom

        for cam_id in range(len(self.cam_names)):
            # 1. Create camera viewmat
            self.viewmat = self.create_camera_viewmat(position, orientation, cam_id)

            # 2. Render numpy image with gsplat
            rgb, alpha, depth = self.render_with_gsplat(viewmats=self.viewmat, Ks=self.intrin_mat)

            # 3. Create and Pub image message
            if self.enable_rgb_pub and rgb is not None and cam_id == 0:
                self.publish_rgb_raw(rgb, stamp_odom, cam_id)
                self.publish_rgb_compressed(rgb, stamp_odom, cam_id)
                # self.publish_rgb_gray(rgb, stamp_odom, cam_id)

            if self.enable_depth_pub and depth is not None and cam_id == 0:
                self.publish_depth_raw(depth, alpha, stamp_odom, cam_id)
                self.publish_depth_compressed(depth, alpha, stamp_odom, cam_id)
                # self.publish_depth_inv(depth, stamp_odom, cam_id)
                # self.publish_depth_pc(depth, self.viewmat, stamp_odom, cam_id)

    def publish_rgb_raw(self, rgb, stamp, cam_id=0):
        if rgb is None: return
        rgb_raw = (rgb * 255).astype(np.uint8)

        # cv_bridge 标准构造：rgb8（gsplat 输出为 RGB 顺序，必须显式声明；passthrough 会推断成 bgr8）
        msg = self.cv_bridge.cv2_to_imgmsg(rgb_raw, encoding="rgb8",
                                           header=Header(stamp=stamp, frame_id="camera"))
        self.rgb_raw_pubs[cam_id].publish(msg)

        rospy.loginfo(f"RGB Raw: Cam{cam_id}, Size: {msg.width}x{msg.height}")

    def publish_rgb_compressed(self, rgb, stamp, cam_id=0):
        if rgb is None: return
        rgb_raw = (rgb * 255).astype(np.uint8)
        bgr = rgb_raw[..., ::-1]  # rgb8 -> bgr8（cv_bridge jpg 压缩需要 BGR）

        msg = self.cv_bridge.cv2_to_compressed_imgmsg(bgr, dst_format='jpg')
        msg.header.stamp = stamp
        msg.header.frame_id = "camera"
        self.rgb_compressed_pubs[cam_id].publish(msg)

        rospy.loginfo_throttle(1.0, f"RGB Compressed: Cam{cam_id}, Size: {bgr.shape[1]}x{bgr.shape[0]}")

    def publish_rgb_gray(self, rgb, stamp, cam_id=0):
        if rgb is None: return
        rgb_gray = np.dot(rgb[..., :3], [0.299, 0.587, 0.114])
        rgb_gray = (rgb_gray * 255).clip(0, 255).astype(np.uint8)

        msg = self.cv_bridge.cv2_to_imgmsg(rgb_gray, encoding="mono8",
                                           header=Header(stamp=stamp, frame_id="camera"))
        self.rgb_gray_pubs[cam_id].publish(msg)

        # rospy.loginfo(f"RGB Gray: Cam{cam_id}, Size: {msg.width}x{msg.height}, "
        #               f"Range: {rgb_gray.min()}-{rgb_gray.max()}, Mean: {rgb_gray.mean():.1f}")

    def publish_depth_raw(self, depth, alpha, stamp, cam_id=0):
        if depth is None:
            return

        depth_float = depth.copy()
        depth_mm = depth_float * 1000.0
        depth_16uc1 = depth_mm.astype(np.uint16)

        min_depth_mm = 100
        max_depth_mm = 20000
        depth_16uc1 = depth_16uc1.clip(min_depth_mm, max_depth_mm)
        depth_16uc1[alpha < self.alpha_threshold] = 0  # alpha 掩码：无效深度置零

        msg = self.cv_bridge.cv2_to_imgmsg(depth_16uc1, encoding="16UC1",
                                           header=Header(stamp=stamp, frame_id="camera"))
        self.depth_raw_pubs[cam_id].publish(msg)

        rospy.loginfo(f"Depth Raw: Cam{cam_id}, Size: {msg.width}x{msg.height}")

    def publish_depth_compressed(self, depth, alpha, stamp, cam_id=0):
        if depth is None:
            return

        depth_float = depth.copy()
        depth_mm = depth_float * 1000.0
        depth_16uc1 = depth_mm.astype(np.uint16)

        min_depth_mm = 100
        max_depth_mm = 20000
        depth_16uc1 = depth_16uc1.clip(min_depth_mm, max_depth_mm).squeeze()
        depth_16uc1[alpha.squeeze() < self.alpha_threshold] = 0  # alpha 掩码：无效深度置零

        msg = self.cv_bridge.cv2_to_compressed_imgmsg(depth_16uc1, dst_format='png')
        msg.data = struct.pack('<iff', 0, 0.0, 0.0) + msg.data  # 12 字节 ConfigHeader
        msg.format = '16UC1; compressedDepth png'
        msg.header.stamp = stamp
        msg.header.frame_id = "camera"
        self.depth_compressed_pubs[cam_id].publish(msg)

        rospy.loginfo_throttle(1.0, f"Depth Compressed: Cam{cam_id}, Size: {depth_16uc1.shape[1]}x{depth_16uc1.shape[0]}")

    def publish_depth_inv(self, depth, stamp, cam_id=0):
        if depth is None: return
        depth_inv = 1.0 - (depth / depth.max()) if depth.max() > 0 else np.zeros_like(depth)
        depth_inv = depth_inv.astype(np.float32)

        msg = self.cv_bridge.cv2_to_imgmsg(depth_inv, encoding="32FC1",
                                           header=Header(stamp=stamp, frame_id="camera"))
        self.depth_inv_pubs[cam_id].publish(msg)

        rospy.loginfo(f"Depth Raw: Inv{cam_id}, Size: {msg.width}x{msg.height}")

    def publish_depth_pc(self, depth, viewmat, stamp, cam_id=0):
        # 深度图转点云
        points = self.depth_to_pointcloud(depth, viewmat, cam_id)
        if points is None or len(points) == 0:
            # rospy.logwarn(f"No points to publish for camera {cam_id}")
            return

        # 创建 PointCloud2 消息
        header = Header()
        header.stamp = stamp
        header.frame_id = "world"
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_data = list(zip(points[:, 0], points[:, 1], points[:, 2]))
        pc_msg = pc2.create_cloud(header, fields, point_data)

        if cam_id < len(self.depth_debug_pubs):
            self.depth_debug_pubs[cam_id].publish(pc_msg)
            # rospy.loginfo(f"Published {len(points)} points from camera {cam_id}")
        else:
            rospy.logwarn(f"No publisher for camera {cam_id}")

    def depth_to_pointcloud(self, depth_np, viewmat, cam_id=0):
        if depth_np is None or viewmat is None:
            return None

        if len(depth_np.shape) == 3:
            depth_np = depth_np.squeeze()
        depth_np = np.fliplr(depth_np)

        H, W = depth_np.shape
        u = np.arange(W)
        v = np.arange(H)
        uu, vv = np.meshgrid(u, v)
        mask = depth_np > 0.1

        # 提取有效像素
        u_valid = uu[mask]
        v_valid = vv[mask]
        depth_valid = depth_np[mask]

        if len(depth_valid) == 0:
            rospy.loginfo(f"No valid depth points for cam{cam_id}")
            return None

        rospy.loginfo(f"Converting {len(depth_valid)} depth points to pointcloud (cam{cam_id})")

        # 使用内参矩阵进行反投影
        X_cam = (u_valid - self.cx) * depth_valid / self.fx
        Y_cam = (v_valid - self.cy) * depth_valid / self.fy
        Z_cam = depth_valid

        # 相机坐标系中的点 [N, 3]
        points_cam = np.column_stack([X_cam, Y_cam, Z_cam])

        # 转换 viewmat 为 numpy
        if torch.is_tensor(viewmat):
            viewmat_np = viewmat.squeeze(0).cpu().numpy()  # [4, 4]
        else:
            viewmat_np = viewmat

        # 计算逆矩阵
        try:
            viewmat_inv = np.linalg.inv(viewmat_np)
        except np.linalg.LinAlgError:
            rospy.logwarn("View matrix is singular, cannot invert")
            return None

        # 相机坐标转换为齐次坐标 [N, 4]
        ones = np.ones((len(points_cam), 1))
        points_cam_homogeneous = np.hstack([points_cam, ones])

        # 应用逆变换：世界坐标 = viewmat_inv * 相机坐标
        points_world_homogeneous = (viewmat_inv @ points_cam_homogeneous.T).T

        # 转换为非齐次坐标 [N, 3]
        points_world = points_world_homogeneous[:, :3]

        # 检查是否有无效值
        valid_mask = np.all(np.isfinite(points_world), axis=1)
        points_world = points_world[valid_mask]

        if len(points_world) == 0:
            rospy.logwarn("All points are invalid after transformation")
            return None
        return points_world


def main():
    parser = argparse.ArgumentParser(description="ROS 3DGS Renderer")
    parser.add_argument("--ply_file", type=Path, default="3dgs.ply")
    parser.add_argument("--params_file", type=Path,
                        default=Path(__file__).resolve().parent.parent / "config" / "renderer_params.yaml")
    parser.add_argument("--enable_gimbal", action="store_true", default=False)
    args = parser.parse_args()

    rospy.init_node('gsplat_renderer')
    GsplatRenderer(ply_file=args.ply_file, params_file=args.params_file, enable_gimbal=args.enable_gimbal)
    rospy.spin()


if __name__ == "__main__":
    main()
