#!/usr/bin/env python3
"""
HDBSCAN Clusterizer takes in a Point Cloud from ROS, clusterizes it by using HBDSCAN
algorithm and creates Bounding Boxes in RViz from the clustered objects.


Author: Henrique Poleselo
November 19th 2022.
"""

import open3d as o3d
import numpy as np
import point_cloud_utils
from typing import List
from collections import deque
import random
from jsk_recognition_msgs.msg import BoundingBox
import rospy

# TODO: Add variable to show in Open3D for offline testing
view_in_open3d = False

boundin_box_publisher = rospy.Publisher('cluster_bounding_boxes', BoundingBox)

rospy.init_node('register')


def configure_default_bounding_box():
    default_bounding_box = BoundingBox()

    # No rotation for the bounding box.
    default_bounding_box.pose.orientation.w = 1
    default_bounding_box.pose.orientation.z = 0
    default_bounding_box.pose.orientation.y = 0
    default_bounding_box.pose.orientation.x = 0

    # TODO: This will change to cluster_N dynamically afterwards so that each cluster has its frame_id
    default_bounding_box.header.frame_id = 'cluster'

    default_bounding_box.pose.position.x = 0.0
    default_bounding_box.pose.position.y = 0.0
    default_bounding_box.pose.position.z = 0.0

    # * Will be dynamically populated
    default_bounding_box.dimensions.x = 0.0
    default_bounding_box.dimensions.y = 0.0
    default_bounding_box.dimensions.z = 0.0

    # * Will be dynamically populated
    default_bounding_box.label = 0
    return default_bounding_box

# TODO: Place them into another module
# ----- Helper Functions ------

def get_points_from_point_cloud(pcd: o3d.geometry.PointCloud()):
    return np.asarray(pcd.points)

def get_point_cloud_size(pcd) -> int:
    return pcd.shape[0]

def get_number_of_labels(labels: list) -> list:
    """
    HDBSCAN will return an array containing the labels, this function creates a unique list out of
    the unique ids.
    """
    return set(labels)

# TODO: use on init to call the configure_default_bounding_box()
default_bounding_box = configure_default_bounding_box()

class PointCloudCluster:
    """
    Class that describes data structure to represent a cluster to be colored in Open3D
    and to generate a bounding box for the cluster. Such bounding box will be published
    to RViz.
    HDBSCAN will output N clusters, therefore N classes should be created.
    """
    label = 0   # Int that is represented by the label from a cluster outputted from HDBSCAN
    color_open3d = []
    cluster_data_points =  []
    bounding_box = default_bounding_box

def generate_clusters(unique_labels: list) -> deque:
    """
    Based on the N unique labels identified from HDBSCAN clustering algorithm, N classes
    will be generated (along with colors) to be populated with the separeted points.
    In other words N different point clouds will be generated and then merged together.
    """

    clusters = deque()
    for label in unique_labels:
        # Generates Red, Green and Blue color channels for the Cluster color
        pcc = PointCloudCluster()
        pcc.label = label
        pcc.color_open3d = [
                            round(random.uniform(0.0, 1.0), 3),    # Red channel
                            round(random.uniform(0.0, 1.0), 3),    # Green channel
                            round(random.uniform(0.0, 1.0), 3)
                            ]
        pcc.cluster_data_points = []
        pcc.bounding_box = default_bounding_box
        
        clusters.append(pcc)
    return clusters

def assign_data_points_to_each_cluster(
                                  point_cloud_clusters: List[PointCloudCluster],
                                  point_cloud,
                                  size_of_pc: int,
                                  unique_labels: List[int], 
                                  labels_from_hdbscan: List[List[float]]
                                  ):
    """
    Compares each data point XYZ from the original Point Cloud and its respective label from the point
    cloud and HDBSCAN output and assigns each data point to the correct cluster (to the correct Class).
    Each class (cluster) will be populated with their respective datapoints, other values are going to be 0. 
    """
    for i in range(size_of_pc):
        for label in unique_labels:
            if labels_from_hdbscan[i] == label:
                # Since we're expecting the labels to be ordered, we can reference the correct class/cluster id by the label itself
                # Stores the datapoint to the class
                point_cloud_clusters[label].cluster_data_points.append(point_cloud[i])

def convert_to_open3d_and_paint_clusters(colored_point_cloud_clusters) -> deque:
    """ 
    1. Gets each separated cluster with its respective data points and creates a new
    PointCloud() class object for each cluster and assigns the points to it.

    2. Then each min/max points of each cluster, which will result in the bounding box computation
    will be calculated and passed as a new element in the same list where we're assigning
    the painted clusters.

    3. Each class object is then painted using Open3D function.

    4. Bounding Boxes from Open3D built-in function are drawn (this won't be needed in the future as
    we'll be drawing everything in RViz.)

    5. As mentined in step 2, both painted clusters, bounding boxes for Open3D and bounding boxes
    points for RViz visualization are passed in to the same list (eases the visualization).

    """
    colored_clusters_with_data_points_and_bboxes = deque()
    bounding_boxes_points = deque()
    for colored_point_cloud_cluster in colored_point_cloud_clusters:
        colored_pc = o3d.geometry.PointCloud()
        colored_pc.points = o3d.utility.Vector3dVector(colored_point_cloud_cluster.data_points)
        
        # Retrieving min and max data points to draw bounding boxes in RViz
        bounding_box_points = point_cloud_utils.get_bounding_box_vertices(colored_pc)
        
        bounding_boxes_points.append(bounding_box_points)

        # Creates new separated point clouds for each bounding box starting point (contains only one XYZ)
        colored_clusters_with_data_points_and_bboxes.append(point_cloud_utils.create_point_cloud_from_bbox_vertices(bounding_box_points))

        colored_pc.paint_uniform_color(colored_point_cloud_cluster.color_open3d)

        # Draw bounding box around the point cloud
        colored_clusters_with_data_points_and_bboxes.append(colored_pc.get_axis_aligned_bounding_box())
        colored_clusters_with_data_points_and_bboxes.append(colored_pc)
    
    # ! Should return BOUNDING BOX POINTS
    return colored_clusters_with_data_points_and_bboxes, bounding_box_points

# * We're gonna be spawning the BoundingBox with default rotation and just populate later
# * With the dimensions and origin points

# TODO: Place everything into a single function

# TODO: improve performance for dealing with class
# TODO: https://stackoverflow.com/questions/44046920/changing-class-attributes-by-reference

def draw_bounding_box_from_cluster(bounding_box_points,
                                   bounding_box_dimensions_xyz,
                                   cluster_label: int):


    # TODO: Change from creating a bouding Box Class to actually get the already-created
    # TODO: class and just update the values

    
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

def function_to_return_n_classes():
    pass

if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud('wall_table_chair.pcd')
    downsampled_pc = point_cloud_utils.downsample(pcd)
    # TODO: Will be replaced by the pass-through filter
    segmented_pc = point_cloud_utils.segment_plane(downsampled_pc)

    cluster_labels_for_each_data_point = point_cloud_utils.cluster_hdbscan(segmented_pc)

    # --- Custom functions to generate classes for each cluster ---
    pcd_points = get_points_from_point_cloud(segmented_pc)

    unique_labels = get_number_of_labels(cluster_labels_for_each_data_point)
    point_cloud_size = get_point_cloud_size(pcd_points)
    colored_clusters = generate_clusters(unique_labels)

    # Updates each class/cluster with given their respective data points
    assign_data_points_to_each_cluster(colored_clusters, pcd_points, point_cloud_size, unique_labels, cluster_labels_for_each_data_point)
    
    open3d_clusterized_colored_pc = convert_to_open3d_and_paint_clusters(colored_clusters)

