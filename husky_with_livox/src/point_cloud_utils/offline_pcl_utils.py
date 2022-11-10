import open3d as o3d
import matplotlib.pyplot as plt
import os
import numpy as np
import point_cloud_utils
import os

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

def read_and_merge_pcd():
    # Adapting for local testing (i.e. reading the PCL as a file, not from ROS)
    test_data_path = "../../test_data/"
    point_clouds_directories = ["table_closer", "table_far_away", "wall_table_chair"]
    list_of_files_path = read_point_cloud_files(test_data_path, point_clouds_directories)
    concatenated_pcd_path = concatenate_multiple_pcds(list_of_files_path)
    print(f"Reading: {concatenated_pcd_path}")
    pcd = o3d.io.read_point_cloud(concatenated_pcd_path)
    return pcd

# Testing Locally
if __name__ == '__main__':
    pcd = read_and_merge_pcd()

    # To get min and max to perform the pass-through filter
    pcd_points = np.asarray(pcd.points)

    # Gets min and max from Point Cloud so that we just apply
    # Filter to the Z-Axis and other are wide enough
    print(min(pcd_points[0]), min(pcd_points[1]), min(pcd_points[2]))
    #print(min(pcd_points[0]))
    #print(max(pcd_points[0]))
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
    point_cloud_utils.clustering_dbscan(segmented_pc)

    
    """new_col, new_pos = point_cloud_utils.draw_guild_lines(filter_boundaries)
    new_data = np.concatenate((new_pos, new_col), axis = 1)
    print(new_col)
    print(new_pos)
neno vai ficar no foco aq
    guild_points = o3d.geometry.PointCloud()
    guild_points.points = o3d.utility.Vector3dVector(new_pos)
    guild_points.colors = o3d.utility.Vector3dVector(new_col)

    o3d.visualization.draw_geometries([guild_points],
                                    zoom=1,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])"""


    """filtered_pcl = point_cloud_utils.pass_through_filter(filter_boundaries, pcd)

    o3d.visualization.draw_geometries([filtered_pcl],
                                    zoom=1,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])"""


    #filtered_pcl = point_cloud_utils.apply_pass_through_filter(pcd, x_range, y_range, z_range)
