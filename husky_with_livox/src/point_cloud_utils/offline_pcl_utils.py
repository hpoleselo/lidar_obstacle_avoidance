import open3d as o3d
import matplotlib.pyplot as plt
import os
import numpy as np
import point_cloud_utils

def concatenate_multiple_pcds(list_of_pcd):
    """
    Concatenates multiple PCDs file into a single one.
    """
    
    # Creates output filename based on the folder of given point cloud
    file_name = list_of_pcd[0].split('/')[3]
    output_file_name = f"{file_name}.pcd"

    # Stores converted pcds to temp. list to be concatenated later
    pcd_t = []

    for pcd_file in list_of_pcd:
        print(f"Reading {pcd_file}")
        pcd = o3d.io.read_point_cloud(pcd_file)
        pcd_load = np.asarray(pcd.points)
        pcd_t.append(pcd_load)
    final_pcd = np.concatenate(pcd_t)
    print(f"Shape of Final PCD: {final_pcd.shape}")
    concatenated_pcd = o3d.geometry.PointCloud()
    concatenated_pcd.points = o3d.utility.Vector3dVector(final_pcd)
    
    # TODO: Convert pcd to open3d 
    
    o3d.io.write_point_cloud(output_file_name, concatenated_pcd)
    print(f"Wrote merged PCD to {output_file_name}")
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

# Testing Locally
if __name__ == '__main__':
    # Adapting for local testing (i.e. reading the PCL as a file, not from ROS)
    test_data_path = "../../test_data/"
    point_clouds_directories = ["table_closer", "table_far_away", "wall_table_chair"]
    list_of_files_path = read_point_cloud_files(test_data_path, point_clouds_directories)
    concatenated_pcd_path = concatenate_multiple_pcds(list_of_files_path)

    print(f"Reading: {concatenated_pcd_path}")
    pcd = o3d.io.read_point_cloud(concatenated_pcd_path)
    # TODO: Add visualization
    o3d.visualization.draw_geometries([pcd],
                                    zoom=0.455,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])