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
from collections import deque
import hdbscan
from jsk_recognition_msgs.msg import BoundingBox

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

def downsample(pcd):
    print("Downsample the point cloud with a voxel of 0.05")
    voxelized_pcd = pcd.voxel_down_sample(voxel_size=0.05)
    return voxelized_pcd

def clustering_dbscan(pcd):
    """
    Clusters a PointCloud in Numpy format using DBSCAN implementation
    from Open3D.
    
    """
    
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.1, min_points=17, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    return pcd
    
def draw_guild_lines(boundaries, density = 0.01):
    """
    Draw wished boundaries for a pass-through filter.
    """
    is_rgb_4byte = False

    new_col = []
    new_pos = []
    x_start,x_end = boundaries["x"]
    y_start,y_end = boundaries["y"]
    z_start,z_end = boundaries["z"]

    x_points,y_points,z_points = np.asarray(np.arange(x_start,x_end,density)),np.asarray(np.arange(y_start,y_end,density)),np.asarray(np.arange(z_start,z_end,density))
    
    y_starts,y_ends = np.asarray(np.full((len(x_points)),y_start)),np.asarray(np.full((len(x_points)),y_end))
    z_starts,z_ends = np.asarray(np.full((len(x_points)),z_start)),np.asarray(np.full((len(x_points)),z_end))
    lines_x = np.concatenate((np.vstack((x_points,y_starts,z_starts)).T,np.vstack((x_points,y_ends,z_starts)).T,np.vstack((x_points,y_starts,z_ends)).T,np.vstack((x_points,y_ends,z_ends)).T))


    x_starts,x_ends = np.asarray(np.full((len(y_points)),x_start)),np.asarray(np.full((len(y_points)),x_end))
    z_starts,z_ends = np.asarray(np.full((len(y_points)),z_start)),np.asarray(np.full((len(y_points)),z_end))
    lines_y = np.concatenate((np.vstack((x_starts,y_points,z_starts)).T,np.vstack((x_ends,y_points,z_starts)).T,np.vstack((x_starts,y_points,z_ends)).T,np.vstack((x_ends,y_points,z_ends)).T))


    x_starts,x_ends = np.asarray(np.full((len(z_points)),x_start)),np.asarray(np.full((len(z_points)),x_end))
    y_starts,y_ends = np.asarray(np.full((len(z_points)),y_start)),np.asarray(np.full((len(z_points)),y_end))
    lines_z = np.concatenate((np.vstack((x_starts,y_starts,z_points)).T,np.vstack((x_ends,y_starts,z_points)).T,np.vstack((x_starts,y_ends,z_points)).T,np.vstack((x_ends,y_ends,z_points)).T))

    if (is_rgb_4byte):
        lines_x_color =  np.full((len(lines_x)),rgb2float(255,0,0))#blue for x
        lines_y_color =  np.full((len(lines_y)),rgb2float(0,255,0))#green for y
        lines_z_color =  np.full((len(lines_z)),rgb2float(0,0,255))#red for z
        return np.concatenate((lines_x,lines_y,lines_z)),np.asmatrix(np.concatenate((lines_x_color,lines_y_color,lines_z_color))).T
    else:
        lines_x_color = np.zeros((len(lines_x),3))
        lines_y_color = np.zeros((len(lines_y),3))
        lines_z_color = np.zeros((len(lines_z),3))

        lines_x_color[:,0] = 1.0 #red for x
        lines_y_color[:,1] = 1.0 #green for y
        lines_z_color[:,2] = 1.0 #blue for z

    return np.concatenate((lines_x,lines_y,lines_z)), np.asmatrix(np.concatenate((lines_x_color,lines_y_color,lines_z_color)))

def get_pass_through_filter_boundaries(point_cloud_points: np.array):
    """
    Based on the input point cloud, get min and max of each dimension (XYZ) and adds
    one small offsets in the Z-axis so that ground plane removal works and outputs
    a dictionary so that it can be used on a pass-through filter.
    """
    x_max, y_max, z_max = point_cloud_points.max(axis=0)
    x_min, y_min, z_min = point_cloud_points.min(axis=0)
    
    # 0.08cm offset
    z_offset_ground_removal = 0.08

    filter_boundaries = {
        "x": [x_min, x_max],
        "y": [y_min, y_max],
        "z": [z_min + z_offset_ground_removal, z_max]
    }

    return filter_boundaries

def pass_through_filter(boundaries, pcd):
    """
    Removes one
    Args:
        boundaries (dict)
    Returns
        o3dpc (open3d.geometry.PointCloud): filtered open3d point cloud
    """
    points = np.asarray(pcd.points)
    #print(f"Input Point Cloud with size of: {len(points)}")
    x_range = np.logical_and(points[:,0] >= boundaries["x"][0] ,points[:,0] <= boundaries["x"][1])
    y_range = np.logical_and(points[:,1] >= boundaries["y"][0] ,points[:,1] <= boundaries["y"][1])
    z_range = np.logical_and(points[:,2] >= boundaries["z"][0] ,points[:,2] <= boundaries["z"][1])
    pass_through_filter = np.logical_and(x_range,np.logical_and(y_range,z_range))
    pcd.points = o3d.utility.Vector3dVector(points[pass_through_filter])
    return pcd

def segment_plane(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                            ransac_n=3,
                                            num_iterations=100)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    """o3d.visualization.draw_geometries([outlier_cloud],
                                    zoom=0.8,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])"""
    return outlier_cloud

def cluster_hdbscan(pcd):
    pcd_points = np.asarray(pcd.points)
    clusterer = hdbscan.HDBSCAN()
    #print(dir(clusterer))
    clusterer.fit(pcd_points)
    print(f"Got: {max(clusterer.labels_)+1} clusters.")
    
    # ! If we wanted to select one epsilon for HDSCAN to compare with DBSCAN:
    #clusters = clusterer.single_linkage_.get_clusters(
                                    # Where this would be the specific epsilon
    #                                cut_distance=0.25,
    #                                min_cluster_size=2
    #                                )
    return clusterer.labels_

def get_bounding_box_vertices_and_dimensions(cluster_point_cloud):
    """
    Gets each cluster min/max in order to draw each cluster a bounding box
    This function will be used by both Open3D and RVIZ.
    """
    pc_points = np.asarray(cluster_point_cloud.points)

    x_max, y_max, z_max = pc_points.max(axis=0)
    x_min, y_min, z_min = pc_points.min(axis=0)

        #[x_min, y_max, z_min]

    bounding_box_points = [
        [x_max, y_max, z_max],
        [x_min, y_min, z_min],
        [x_min, y_min, z_max],  # Varies only Z from the previous element, meaning we get Z
        [x_min, y_max, z_min],   # Varies only Y from the element 1, meaning we get Y
        [x_max, y_min, z_min]
    ]

    bounding_box_points = np.array(bounding_box_points)

    box_dimensions_xyz = calculate_bounding_box_dimensions(bounding_box_points)
    # Prior testing
    #x_y_z_max = [x_max, y_max, z_max]
    #x_y_z_min = [x_min, y_min, z_min]
    #print("\nPoint Clound Boundaries from Numpy:")
    #print(f"Min: {x_y_z_min}\nMax:{x_y_z_max}")
    #bounding_box = cluster_point_cloud.get_axis_aligned_bounding_box()
    #print(f"\nBounding Box From Open3D: {np.asarray(bounding_box)}")

    return bounding_box_points, box_dimensions_xyz

def calculate_bounding_box_dimensions(bounding_box_points):
    """
    Given the identified cluster max/min data points (i.e: vertices for the
    to-be drawn bounding box), calculates the X, Y and Z dimensions of the box
    in order to fit the clustered object.

    Such operation is needed because BoundingBox.msg from jsk_recognition_msgs.msg
    takes in such parameters in order to draw the box in RViz.
    """
    bbox_z_dimension = bounding_box_points[2] - bounding_box_points[1]
    bbox_y_dimension = bounding_box_points[3] - bounding_box_points[1]
    bbox_x_dimension = bounding_box_points[4] - bounding_box_points[1]
    box_dimensions_xyz = (bbox_x_dimension[0], bbox_y_dimension[1], bbox_z_dimension[2])
    return box_dimensions_xyz

def create_point_cloud_from_bbox_vertices(bounding_box_points):
    bounding_box_spawn_point_cloud= o3d.geometry.PointCloud()
    # Has to be (N, 3)
    bbox_origin = np.asarray(bounding_box_points)
    bounding_box_spawn_point_cloud.points = o3d.utility.Vector3dVector(bbox_origin)
    # Point will be red
    bounding_box_spawn_point_cloud.paint_uniform_color([1.0, 0.0, 0.0])
    return bounding_box_spawn_point_cloud

def draw_bounding_box_from_cluster(bounding_box_points,
                                   bounding_box_dimensions_xyz,
                                   cluster_label: int):
    # Represents cluster initial point XYZ wrt. to Lidar
    bounding_box_origin = bounding_box_points[0]

    bbox = BoundingBox()
    bbox.header.frame_id = 'cluster'
    bbox.pose.position.x = bounding_box_origin[0]
    bbox.pose.position.y = bounding_box_origin[1]
    bbox.pose.position.z = bounding_box_origin[2]

    # No rotation for the bounding box.
    bbox.pose.orientation.w = 1
    bbox.pose.orientation.z = 0
    bbox.pose.orientation.y = 0
    bbox.pose.orientation.x = 0

    bbox.dimensions.x = bounding_box_dimensions_xyz[0]
    bbox.dimensions.y = bounding_box_dimensions_xyz[1]
    bbox.dimensions.z = bounding_box_dimensions_xyz[2]

    bbox.label = cluster_label
    boundin_box_publisher.publish(bbox)

# Testing Locally
if __name__ == '__main__':
    # Adapting for local testing (i.e. reading the PCL as a file, not from ROS)
    test_data_path = "../../test_data/"
    #print(f"Reading: {path_to_test_file}")
    #pcd = o3d.io.read_point_cloud(concatenated_pcd_path)