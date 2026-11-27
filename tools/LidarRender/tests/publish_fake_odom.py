#!/usr/bin/env python3
"""Publish a fixed nav_msgs/Odometry pose for LidarRender smoke tests."""

import argparse
import math

import rospy
from nav_msgs.msg import Odometry


def parse_args():
    parser = argparse.ArgumentParser(description="Publish a fixed odometry pose")
    parser.add_argument("--topic", default="/super/sim/lidar_slam/odom")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.5)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--frame-id", default="world")
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("lidar_fake_odom", anonymous=True)
    publisher = rospy.Publisher(args.topic, Odometry, queue_size=10)
    rate = rospy.Rate(args.rate)
    message = Odometry()
    message.header.frame_id = args.frame_id
    message.child_frame_id = "lidar_render"
    message.pose.pose.position.x = args.x
    message.pose.pose.position.y = args.y
    message.pose.pose.position.z = args.z
    message.pose.pose.orientation.z = math.sin(args.yaw * 0.5)
    message.pose.pose.orientation.w = math.cos(args.yaw * 0.5)

    while not rospy.is_shutdown():
        message.header.stamp = rospy.Time.now()
        publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
