import unittest

import numpy as np
import rospy
from sensor_msgs.msg import PointField

from cloud2 import cloud2_from_xyz_i


class Cloud2Test(unittest.TestCase):
    def test_builds_pointcloud2(self):
        points = np.array([[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]], dtype="<f4")
        stamp = rospy.Time(1, 2)
        message = cloud2_from_xyz_i(points, stamp, "world")
        self.assertEqual(message.width, 2)
        self.assertEqual(message.height, 1)
        self.assertEqual(message.point_step, 16)
        self.assertEqual(message.row_step, 32)
        self.assertEqual(message.header.stamp, stamp)
        self.assertEqual(message.header.frame_id, "world")
        self.assertFalse(message.is_bigendian)
        self.assertTrue(message.is_dense)
        self.assertEqual(message.data, points.tobytes())
        self.assertEqual([field.name for field in message.fields],
                         ["x", "y", "z", "intensity"])
        self.assertEqual([field.offset for field in message.fields], [0, 4, 8, 12])
        self.assertEqual([field.datatype for field in message.fields],
                         [PointField.FLOAT32] * 4)

    def test_marked_not_dense_for_non_finite(self):
        points = np.array([[1.0, 2.0, 3.0, np.nan]], dtype="<f4")
        message = cloud2_from_xyz_i(points, rospy.Time(0, 0), "world")
        self.assertFalse(message.is_dense)

    def test_empty_cloud(self):
        points = np.empty((0, 4), dtype="<f4")
        message = cloud2_from_xyz_i(points, rospy.Time(0, 0), "world")
        self.assertEqual(message.width, 0)
        self.assertEqual(message.row_step, 0)
        self.assertEqual(message.data, b"")


if __name__ == "__main__":
    unittest.main()
