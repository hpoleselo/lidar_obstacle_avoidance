#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud
from point_cloud_utils.pointcloud_utils import ros_to_numpy

def pointcloud_callback(data):
    # For testing purposes
    #rospy.loginfo(f"{rospy.get_caller_id()} + I Heard {data.points}")
    xyz = ros_to_numpy(data)
    #downsample(xyz)

def point_cloud_listener():
    rospy.init_node('pointcloud_converter', anonymous=True)
    rospy.Subscriber("scan", PointCloud, pointcloud_callback)
    # spin() simply keeps python from exiting until this node is stopped
    # This may affect Open3D visualization in real-time, if wished.
    rospy.spin()

if __name__ == '__main__':
    point_cloud_listener()
