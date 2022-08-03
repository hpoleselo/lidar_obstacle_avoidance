import numpy as np
import open3d as o3d
from sys import exit

if __name__ == "__main__":

    print("\n----- Load a PCD point cloud, print it, and render it -----")
    pcd = o3d.io.read_point_cloud("../table_scene_lms400.pcd")
    if pcd.is_empty():
        exit()
    print(pcd)

    # <class 'open3d.cpu.pybind.geometry.PointCloud'>
    # We can explore several methods to deal with point clouds already built in in this class
    print(type(pcd))

    # Convert the PCL to an array, which is a matrix
    # We expect an array in the format of (NumberOfPoints, 3)
    # As we have three coordinates for each point
    print(np.asarray(pcd.points).shape)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])
    """Using a default view when opening the visualization
                                      zoom=0.9,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024]"""

    print("\n----- Downsample the point cloud with a voxel of 0.05 -----")
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    print(f"Downsampled PCL: \n {np.asarray(downpcd.points).shape}")
    o3d.visualization.draw_geometries([downpcd])

    print("\n----- Recompute the normal of the downsampled point cloud -----")
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))

    print("Now the downsampled point cloud has more information (the normals), which indicates the rotation of the point.")
    o3d.visualization.draw_geometries([downpcd])

    print(f"Downsampled PCL with normal vectors incorporated have the same shape as the downsampled PCL:\n{np.asarray(downpcd.normals).shape}")
    print(f"Print a normal vector of the 0th point of the point cloud:\n{downpcd.normals[0]}")
    print(f"Print the normal vectors of the first 10 points:\n{np.asarray(downpcd.normals)[:10, :]}")