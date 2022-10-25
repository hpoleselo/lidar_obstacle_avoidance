#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
import numpy as np

# Subscriber to take in a PointCloud msg, converts to Open3D and downsample it

def ros_to_numpy(ros_pointcloud_msg):
    """
    Converts sensor_msgs/PointCloud.msg to numpy array.
    http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud.html
    """
    xyz = []

    print(ros_pointcloud_msg.points[0])

    for data in ros_pointcloud_msg.points:

        xyz.append([data.x, data.y, data.z])
        #rgb = np.append(rgb,[[r,g,b]], axis = 0)	

    return np.array(xyz)

def downsample():
    """
    Converts numpy array to Open3D class and downsamples it.
    """
    pass

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # ? Do we do the downsampling immediately after receiving the data?
    # ? Or should we buffer it?
    #xyz = ros_to_numpy(data)
    #downsample(xyz)

def point_cloud_listener():
    rospy.init_node('pointcloud_converter', anonymous=True)
    rospy.Subscriber("scan", PointCloud, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    point_cloud_listener()
