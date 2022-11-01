import numpy as np
from sensor_msgs.msg import PointCloud, PointField
from std_msgs.msg import Header
import rospy
from struct import pack, unpack
import ctypes

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

def numpy_to_ros(numpy_array):
    """ Converts a numpy array used in Open3D to a ROS sensor_msgs/PointCloud.msg """
    pcl_msg = PointCloud()
    pcl_msg.header.stamp = rospy.Time.now()
    pcl_msg.header.frame_id = "world"

    pcl_msg.height = 1
    pcl_msg.width = numpy_array.size

    pcl_msg.fields.append(PointField(
                            name="x",
                            offset=0,
                            datatype=PointField.FLOAT32, count=1))
    pcl_msg.fields.append(PointField(
                            name="y",
                            offset=4,
                            datatype=PointField.FLOAT32, count=1))
    pcl_msg.fields.append(PointField(
                            name="z",
                            offset=8,
                            datatype=PointField.FLOAT32, count=1))
    pcl_msg.fields.append(PointField(
                            name="rgb",
                            offset=16,
                            datatype=PointField.FLOAT32, count=1))

    pcl_msg.is_bigendian = False
    pcl_msg.point_step = 32
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width * pcl_msg.height
    pcl_msg.is_dense = False
    
    buffer_t = []

    for data in numpy_array:
        # * Converts Python values to C srtucts represented as bytes Python objects 
        s = pack('>f', data[3])

        # ! This probably won't be needed as we're not getting a colored PCL
        i = unpack('>l', s)[0]
        packed = ctypes.c_uint32(i).value

        r = (packed & 0x00FF0000) >> 16
        g = (packed & 0x0000FF00) >> 8
        b = (packed & 0x000000FF)

        # ! We would fill with empty values for RGB
        buffer_t.append(pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

    pcl_msg.data = "".join(buffer_t)

    return pcl_msg

def downsample():
    """
    Converts numpy array to Open3D class and downsamples it.
    """
    pass
