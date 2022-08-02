import numpy as np
import open3d as o3d
from sys import exit

if __name__ == "__main__":

    print("----- Load a PCD point cloud, print it, and render it -----")
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

    print("----- Downsample the point cloud with a voxel of 0.05 -----")
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    print(f"Downsampled PCL: \n {np.asarray(downpcd.points).shape}")
    o3d.visualization.draw_geometries([downpcd])

    print("----- Recompute the normal of the downsampled point cloud -----")
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))
    o3d.visualization.draw_geometries([downpcd])

    print("Print a normal vector of the 0th point")
    print(downpcd.normals[0])
    print("Print the normal vectors of the first 10 points")
    print(np.asarray(downpcd.normals)[:10, :])
    print("")

    print("----- Load a polygon volume and use it to crop the original point cloud -----")
    vol = o3d.visualization.read_selection_polygon_volume(
        "../../TestData/Crop/cropped.json")
    chair = vol.crop_point_cloud(pcd)
    o3d.visualization.draw_geometries([chair])
    print("")

    print("----- Paint chair -----")
    chair.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([chair])
    print("")
