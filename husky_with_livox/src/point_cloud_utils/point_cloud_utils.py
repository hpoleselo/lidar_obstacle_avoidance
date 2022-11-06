import numpy as np
from sensor_msgs.msg import PointCloud, PointField
from std_msgs.msg import Header
import rospy
from struct import pack, unpack
import ctypes
import open3d as o3d
import matplotlib.pyplot as plt
import copy
import os
import sys

def ros_msg_to_numpy(ros_pointcloud_msg):
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

def numpy_to_ros_msg(numpy_array):
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

def clustering_dbscan(pcd):
    """
    Clusters a PointCloud in Numpy format using DBSCAN implementation
    from Open3D.
    
    """
    
    print("Downsample the point cloud with a voxel of 0.05")
    pcd = pcd.voxel_down_sample(voxel_size=0.05)

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.05, min_points=5, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd],
                                    zoom=0.455,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])


# Testing Locally
if __name__ == '__main__':
    # Adapting for local testing (i.e. reading the PCL as a file, not from ROS)
    test_data_path = "../../test_data/"
    #print(f"Reading: {path_to_test_file}")
    #pcd = o3d.io.read_point_cloud(concatenated_pcd_path)