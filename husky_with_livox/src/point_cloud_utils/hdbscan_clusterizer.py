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
    box_dimensions_xyz = [bbox_x_dimension[0], bbox_y_dimension[1], bbox_z_dimension[2]]
    return np.array(box_dimensions_xyz)

def get_cluster_bounding_box_vertices_and_dimensions(cluster_point_cloud):
    """
    Gets each cluster min/max in order to draw each cluster a bounding box.
    This function will be used by both Open3D and RVIZ.

    Returns:
    - bounding_box_origin_points: XYZ point to represent where the initial point will be drawn.
    - box_dimensions_xyz: X, Y and Z dimensions of the bounding box.
    - bounding_box_points: vertices in order to validate if the bounding box is correct in Open3D.
    """
    #pc_points = np.asarray(cluster_point_cloud.points)

    cluster_point_cloud = np.asarray(cluster_point_cloud)

    x_max, y_max, z_max = cluster_point_cloud.max(axis=0)
    x_min, y_min, z_min = cluster_point_cloud.min(axis=0)

        #[x_min, y_max, z_min]

    # * Userd for Open3D in order to validate the bounding box vertices
    bounding_box_points = [
        [x_max, y_max, z_max],
        [x_min, y_min, z_min],
        [x_min, y_min, z_max],  # Varies only Z from the previous element, meaning we get Z
        [x_min, y_max, z_min],  # Varies only Y from the element 1, meaning we get Y
        [x_max, y_min, z_min]
    ]

    bounding_box_origin_points = [x_min, y_min, z_min]
    bounding_box_origin_position = np.array(bounding_box_origin_points)

    bounding_box_points = np.array(bounding_box_points)

    box_dimensions_xyz = calculate_bounding_box_dimensions(bounding_box_points)
    # Prior testing
    #bounding_box = cluster_point_cloud.get_axis_aligned_bounding_box()
    #print(f"\nBounding Box From Open3D: {np.asarray(bounding_box)}")

    return bounding_box_origin_position, box_dimensions_xyz, bounding_box_points

def get_cluster_bounding_box(point_cloud_clusters):
    """
    For each cluster get its bounding box parameters and return as a list.
    """
    bbox_origin_positions = deque()
    bbox_dimensions_xyz = deque()
    for point_cloud_cluster in point_cloud_clusters:
        # Retrieving min and max data points to draw bounding boxes in RViz
        bounding_box_origin_position, box_dimensions_xyz, _ = get_cluster_bounding_box_vertices_and_dimensions(point_cloud_cluster.cluster_data_points)
        bbox_origin_positions.append(bounding_box_origin_position)
        bbox_dimensions_xyz.append(box_dimensions_xyz)
    
    return bbox_origin_positions, bbox_dimensions_xyz

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
    
    bbox_origin_positions, bbox_dimensions_xyz = get_cluster_bounding_box(colored_clusters)
    
    #update_classes
    #draw_bounding_box_from_cluster()

