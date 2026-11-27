"""ROS 1 bridge for LidarRender: odom -> TCP pose, TCP cloud -> PointCloud2."""

import argparse
import socket
import sys
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from bridge_helpers import (
    cloud_payload_to_array,
    load_global_cloud_or_none,
    pose_to_header,
)
from cloud2 import cloud2_from_xyz_i
from tcp_protocol import MESSAGE_DEADLINE_SECONDS, receive_message, send_message

RECONNECT_SECONDS = 1.0
THREAD_JOIN_SECONDS = 3.0


class LidarBridge:
    def __init__(self, host="127.0.0.1", pose_port=19082, cloud_port=19083,
                 odom_topic="/super/sim/lidar_slam/odom",
                 local_topic="/super/sim/cloud_registered",
                 global_topic="/super/sim/global_pc",
                 pcd_file="/data/scene.pcd", frame_id="world"):
        self.host = host
        self.pose_port = int(pose_port)
        self.cloud_port = int(cloud_port)
        self.odom_topic = odom_topic
        self.local_topic = local_topic
        self.global_topic = global_topic
        self.pcd_file = pcd_file
        self.frame_id = frame_id

        self.stop_event = threading.Event()
        self.lifecycle_lock = threading.Lock()
        self.started = False
        self.stopped = False

        self.pose_lock = threading.Lock()
        self.pose_slot = None
        self.pose_event = threading.Event()
        self.pose_socket = None
        self.pose_seq = 0

        self.frame_socket = None

        self.global_lock = threading.Lock()
        self.global_cloud = None
        self.global_load_attempted = False
        self.last_global_connections = 0

        self.local_publisher = rospy.Publisher(
            local_topic, PointCloud2, queue_size=1)
        self.global_publisher = rospy.Publisher(
            global_topic, PointCloud2, queue_size=1, latch=False)
        self.odom_subscriber = rospy.Subscriber(
            odom_topic, Odometry, self.odom_callback)
        self.global_timer = rospy.Timer(rospy.Duration(0.5), self.publish_global)
        self.threads = []

    def start(self):
        with self.lifecycle_lock:
            if self.started:
                return
            if self.stopped:
                raise RuntimeError("bridge already stopped")
            self.started = True
            self.threads = [
                threading.Thread(target=self.pose_loop, name="pose-send", daemon=True),
                threading.Thread(target=self.cloud_loop, name="cloud-receive", daemon=True),
            ]
            for thread in self.threads:
                thread.start()

    def run(self):
        self.start()
        try:
            rospy.spin()
        finally:
            self.stop()

    def stop(self):
        with self.lifecycle_lock:
            if self.stopped:
                return
            self.stopped = True
        self.stop_event.set()
        self.pose_event.set()

        self.close_socket(self.pose_socket)
        self.pose_socket = None
        self.close_socket(self.frame_socket)
        self.frame_socket = None

        self.global_timer.shutdown()
        self.odom_subscriber.unregister()
        self.local_publisher.unregister()
        self.global_publisher.unregister()

        for thread in list(self.threads):
            if thread is not threading.current_thread():
                thread.join(timeout=THREAD_JOIN_SECONDS)

    @staticmethod
    def close_socket(sock):
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def odom_callback(self, message):
        with self.pose_lock:
            self.pose_slot = message
        self.pose_event.set()

    def pose_loop(self):
        while not self.stop_event.is_set():
            try:
                sock = socket.create_connection(
                    (self.host, self.pose_port), timeout=3.0)
            except OSError:
                self.stop_event.wait(RECONNECT_SECONDS)
                continue
            self.pose_socket = sock
            try:
                while not self.stop_event.is_set():
                    if not self.pose_event.wait(0.5):
                        continue
                    self.pose_event.clear()
                    with self.pose_lock:
                        message = self.pose_slot
                        self.pose_slot = None
                    if message is None:
                        continue
                    self.pose_seq += 1
                    header = pose_to_header(message, self.pose_seq)
                    send_message(sock, header, timeout=MESSAGE_DEADLINE_SECONDS)
            except (OSError, TimeoutError):
                pass
            finally:
                self.close_socket(sock)
                self.pose_socket = None
            if not self.stop_event.is_set():
                self.stop_event.wait(RECONNECT_SECONDS)

    def cloud_loop(self):
        while not self.stop_event.is_set():
            try:
                sock = socket.create_connection(
                    (self.host, self.cloud_port), timeout=3.0)
            except OSError:
                self.stop_event.wait(RECONNECT_SECONDS)
                continue
            self.frame_socket = sock
            sock.settimeout(MESSAGE_DEADLINE_SECONDS + 0.5)
            try:
                while not self.stop_event.is_set():
                    try:
                        header, payload = receive_message(sock)
                    except TimeoutError:
                        continue
                    except (EOFError, ValueError, OSError):
                        break
                    try:
                        points = cloud_payload_to_array(header, payload)
                        stamp = rospy.Time(int(header["stamp_sec"]),
                                           int(header["stamp_nsec"]))
                        message = cloud2_from_xyz_i(points, stamp, header["frame_id"])
                        self.local_publisher.publish(message)
                    except (KeyError, ValueError) as error:
                        rospy.logwarn("丢弃非法 cloud 帧: %s", error)
            finally:
                self.close_socket(sock)
                self.frame_socket = None
            if not self.stop_event.is_set():
                self.stop_event.wait(RECONNECT_SECONDS)

    def publish_global(self, event):
        connections = self.global_publisher.get_num_connections()
        if connections == 0:
            self.last_global_connections = 0
            return
        with self.global_lock:
            if self.global_cloud is None and not self.global_load_attempted:
                self.global_load_attempted = True
                self.global_cloud = load_global_cloud_or_none(self.pcd_file)
                if self.global_cloud is None:
                    rospy.logwarn("全局 PCD 不可用，禁用全局点云: %s", self.pcd_file)
                    return
            if self.global_cloud is None:
                return
            if connections == self.last_global_connections:
                return
            message = cloud2_from_xyz_i(
                self.global_cloud, rospy.Time.now(), self.frame_id)
            self.global_publisher.publish(message)
            self.last_global_connections = connections


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="LidarRender ROS 1 bridge")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--pose-port", type=int, default=19082)
    parser.add_argument("--cloud-port", type=int, default=19083)
    parser.add_argument("--odom-topic", default="/super/sim/lidar_slam/odom")
    parser.add_argument("--local-cloud-topic", default="/super/sim/cloud_registered")
    parser.add_argument("--global-cloud-topic", default="/super/sim/global_pc")
    parser.add_argument("--pcd-file", default="/data/scene.pcd")
    parser.add_argument("--frame-id", default="world")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rospy.init_node("lidar_render_bridge", anonymous=True)
    bridge = LidarBridge(
        host=args.host,
        pose_port=args.pose_port,
        cloud_port=args.cloud_port,
        odom_topic=args.odom_topic,
        local_topic=args.local_cloud_topic,
        global_topic=args.global_cloud_topic,
        pcd_file=args.pcd_file,
        frame_id=args.frame_id,
    )
    rospy.on_shutdown(bridge.stop)
    try:
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        bridge.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
