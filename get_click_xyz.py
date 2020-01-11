#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import tf
from geometry_msgs.msg import PointStamped


class Get3Dcoordinates(object):
    """Read a point cloud snapshot and returns the XYZ coordinates of a corresponding pixel location"""

    def __init__(self, rgb_h, rgb_v):
        self.rgb_h = rgb_h
        self.rgb_v = rgb_v
        self.found_3d = False
        # Subscribe point cloud
        global sub_once
        sub_once = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2,
                                    self.find_xyz)
        # Wait until connection
        rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, timeout=100.0)
    
    def find_xyz(self, data):
        # do processing here
        gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=False, uvs=[[self.rgb_h, self.rgb_v]])
        target_xyz_cam = list(gen)

        # do conversion to global coordinate here
        listener = tf.TransformListener()
        listener.waitForTransform("map", "head_rgbd_sensor_rgb_frame", rospy.Time(0), rospy.Duration(4.0))
        rgbd_point = PointStamped()
        rgbd_point.header.frame_id = "head_rgbd_sensor_rgb_frame"
        rgbd_point.header.stamp = rospy.Time(0)
        rgbd_point.point.x = target_xyz_cam[0][0]
        rgbd_point.point.y = target_xyz_cam[0][1]
        rgbd_point.point.z = target_xyz_cam[0][2]
        self.map_point = listener.transformPoint("map", rgbd_point)

        self.found_3d = True
        sub_once.unregister()
