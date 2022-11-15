import open3d as o3d
import matplotlib.pyplot as plt
import os
import numpy as np
import point_cloud_utils
import os
from pydantic import BaseModel
from typing import List
from collections import deque
import random

def visualize_pcd(pcd: list):
    """
    Visualize it in Open3D interface.
    pcd: list. Must be passed in a list format, if wished multiple
    point clouds can be passe.  
    """
    o3d.visualization.draw_geometries(pcd,
                                    zoom=0.49,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])

def concatenate_multiple_pcds(list_of_pcd):
    """
    Concatenates multiple PCDs file into a single one.
    """
    
    # Creates output filename based on the folder of given point cloud
    file_name = list_of_pcd[0].split('/')[3]
    output_file = f"{file_name}.pcd"

    output_file_name = os.path.join(
                        # Root folder path
                        # TODO: Inherit variable to make it dynamic
                        list_of_pcd[0].partition('47')[0],
                        output_file)

    # Stores converted pcds to temp. list to be concatenated later
    pcd_t = []

    for pcd_file in list_of_pcd:
        print(pcd_file)
        if pcd_file == output_file_name:
            continue
        print(f"Reading {pcd_file}")
        pcd = o3d.io.read_point_cloud(pcd_file)
        pcd_load = np.asarray(pcd.points)
        print(f"Shape of pc: {len(pcd_load)}")
        pcd_t.append(pcd_load)
    final_pcd = np.concatenate(pcd_t)
    print(f"Shape of Final PCD: {final_pcd.shape}")
    concatenated_pcd = o3d.geometry.PointCloud()
    concatenated_pcd.points = o3d.utility.Vector3dVector(final_pcd)
    o3d.io.write_point_cloud(output_file_name, concatenated_pcd)
    print(f"Wrote merged PCD to {output_file}")
    return output_file_name

def read_point_cloud_files(test_data_path: list, directories: list) -> list:
    """
    Traverse to wished directory and return list of path to files.
    """
    from os import walk
    list_of_files_path = []
    try:
        for (dirpath, dirnames, filenames) in walk(test_data_path):
            if "wall_table_chair" in dirpath:
                for file_name in filenames:
                    # Skipping all files that are not a .pcd
                    if not file_name.endswith(".pcd"):
                        pass
                    else:
                        list_of_files_path.append(os.path.join(dirpath, file_name))
        return list_of_files_path
    except ValueError as e:
        print(f"{e} due to wrong or non-existing file name.")

def read_and_merge_pcd(test_data_path: str,
                       point_cloud_directories: str
                       ):
    # Adapting for local testing (i.e. reading the PCL as a file, not from ROS)
    list_of_files_path = read_point_cloud_files(test_data_path, point_cloud_directories)
    concatenated_pcd_path = concatenate_multiple_pcds(list_of_files_path)
    print(f"Reading: {concatenated_pcd_path}")
    pcd = o3d.io.read_point_cloud(concatenated_pcd_path)
    return pcd

class ColoredPointCloudCluster(BaseModel):
    """
    Class that describes data structure to represent a cluster to be colored.
    HDBSCAN will output N clusters, therefore N classes should be created 
    """
    label: int
    color_open3d: List[float]
    data_points: List[float]

def get_number_of_labels(labels: list) -> list:
    """
    HDBSCAN will return an array containing the labels, this function creates a unique list out of
    the unique ids.
    """
    return set(labels)

def get_pc_size(pcd) -> int:
    print(pcd)
    print(type(pcd))
    return pcd.shape[0]

def generate_colored_clusters(unique_labels: list) -> deque:
    """
    Based on the N unique labels identified from HDBSCAN clustering algorithm, N classes
    will be generated (along with colors) to be populated with the separeted points.
    In other words N different point clouds will be generated and then merged together.
    """

    clusters = deque()
    for label in unique_labels:
        # Generates Red, Green and Blue color channels for the Cluster color
        cpcc = ColoredPointCloudCluster(
            label = label,
            color_open3d = [
                            round(random.uniform(0.0, 1.0), 3),    # Red channel
                            round(random.uniform(0.0, 1.0), 3),    # Green channel
                            round(random.uniform(0.0, 1.0), 3)],   # Blue channel
            data_points = []
        )
        clusters.append(cpcc)
    return clusters

def assign_data_points_to_colored_cluster(
                                  colored_clusters: List[ColoredPointCloudCluster],
                                  point_cloud,
                                  size_of_pc: int,
                                  unique_labels: List[int], 
                                  labels_from_hdbscan: List[List[float]]
                                  ):
    """
    Compares each data point XYZ and its respective label from the point cloud and HDBSCAN output and
    assigns each data point to the correct cluster (to the correct Class).
    Each class (cluster) will be populated with their respective datapoints, other values are going to be 0. 
    """
    for i in range(size_of_pc):
        for label in unique_labels:
            if labels_from_hdbscan[i] == label:
                # Since we're expecting the labels to be ordered, we can reference the correct class/cluster id by the label itself
                # Stores the datapoint to the class
                colored_clusters[label].data_points.append(point_cloud[i])

def convert_to_open3d_and_paint_clusters(colored_point_cloud_clusters) -> deque:
    colored_clusters_with_data_points = deque()
    for colored_point_cloud_cluster in colored_point_cloud_clusters:
        colored_pc = o3d.geometry.PointCloud()
        colored_pc.points = o3d.utility.Vector3dVector(colored_point_cloud_cluster.data_points)
        colored_pc.paint_uniform_color(colored_point_cloud_cluster.color_open3d)
        colored_clusters_with_data_points.append(colored_pc)
    return colored_clusters_with_data_points

# Testing Locally
if __name__ == '__main__':
    test_data_path = "../../test_data/"
    point_clouds_directories = ["table_closer", "table_far_away", "wall_table_chair"]
    pcd = read_and_merge_pcd(test_data_path, point_clouds_directories)

    # To get min and max to perform the pass-through filter
    pcd_points = np.asarray(pcd.points)

    # Gets min and max from Point Cloud so that we just apply
    # Filter to the Z-Axis and other are wide enough
    #print(min(pcd_points[0]), min(pcd_points[1]), min(pcd_points[2]))
    # Sensor is at least 0.3m above the ground
    #x_range = [-3.5, 3.5]
    #y_range = [-3.5, 3.5]
    #z_range = [-3.5, 3.5]


    x_range = [-3.5, 3 + max(pcd_points[0])]
    y_range = [0, 3 + max(pcd_points[1])]
    z_range = [min(pcd_points[2]) - 0.5, 3 + max(pcd_points[2])]
    
    filter_boundaries = {
        "x": x_range,
        "y": y_range,
        "z": z_range
    }

    downsampled_pcd = point_cloud_utils.downsample(pcd)

    segmented_pc = point_cloud_utils.segment_plane(downsampled_pcd)

    # TODO: Test without segmenting the ground
    cluster_labels_for_each_data_point = point_cloud_utils.cluster_hdbscan(segmented_pc)

    pc = np.asarray(segmented_pc.points)
    pc_size = get_pc_size(pc)
    unique_labels = get_number_of_labels(cluster_labels_for_each_data_point)
    colored_clusters = generate_colored_clusters(unique_labels)
    assign_data_points_to_colored_cluster(colored_clusters, pc, pc_size, unique_labels, cluster_labels_for_each_data_point)
    open3d_clusterized_colored_pc = convert_to_open3d_and_paint_clusters(colored_clusters)
    visualize_pcd(open3d_clusterized_colored_pc)

    #point_cloud_utils.clustering_dbscan(segmented_pc)
