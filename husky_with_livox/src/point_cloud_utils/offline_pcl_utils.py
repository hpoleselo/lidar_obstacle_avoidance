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
import k_means_clustering

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


# Testing Locally
if __name__ == '__main__':

    point_cloud_needs_post_processing = False

    if point_cloud_needs_post_processing:
        test_data_path = "../../test_data/"
        point_clouds_directories = ["table_closer", "table_far_away", "wall_table_chair"]
        pcd = read_and_merge_pcd(test_data_path, point_clouds_directories)
    else:
        pcd = o3d.io.read_point_cloud('wall_table_chair.pcd')

    # To get min and max to perform the pass-through filter
    pcd_points = np.asarray(pcd.points)

    # Gets min and max from Point Cloud so that we just apply
    # Filter to the Z-Axis and other are wide enough
    #print(min(pcd_points[0]), min(pcd_points[1]), min(pcd_points[2]))
    # Sensor is at least 0.3m above the ground
    #x_range = [-3.5, 3.5]
    #y_range = [-3.5, 3.5]
    #z_range = [-3.5, 3.5]

    downsampled_pc = point_cloud_utils.downsample(pcd)

    filter_boundaries = point_cloud_utils.get_pass_through_filter_boundaries(pcd_points)
    filtered_pc = point_cloud_utils.pass_through_filter(filter_boundaries, downsampled_pc)
    visualize_pcd([filtered_pc])
    #segmented_pc = point_cloud_utils.segment_plane(downsampled_pc)

    #clf = k_means_clustering.KMeans(k=3)
    #pc = np.asarray(segmented_pc.points)
    #labels = clf.predict(pc)
    
    #k_means_clustering.draw_labels_on_model(segmented_pc,labels)

    """cluster_labels_for_each_data_point = point_cloud_utils.cluster_hdbscan(segmented_pc)
    visualize_pcd(open3d_clusterized_colored_pc)"""

    #point_cloud_utils.clustering_dbscan(segmented_pc)