import numpy as np
import open3d as o3d
from sys import exit

if __name__ == "__main__":

    print("\n----- Load a ply point cloud, print it, and render it -----")
    pcd = o3d.io.read_point_cloud("./fragment.ply")
    if pcd.is_empty():
        exit()
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    o3d.visualization.draw_geometries([downpcd])

    print("----- Load a polygon volume and use it to crop the original point cloud -----")
    print("Load a polygon volume and use it to crop the original point cloud")

    # Only works in open3d==0.15.0, which is available in newest pip version
    """demo_crop_data = o3d.data.DemoCropPointCloud()
    pcd = o3d.io.read_point_cloud(demo_crop_data.point_cloud_path)
    vol = o3d.visualization.read_selection_polygon_volume(demo_crop_data.cropped_json_path)
    chair = vol.crop_point_cloud(pcd)
    o3d.visualization.draw_geometries([chair],
                                    zoom=0.7,
                                    front=[0.5439, -0.2333, -0.8060],
                                    lookat=[2.4615, 2.1331, 1.338],
                                    up=[-0.1781, -0.9708, 0.1608])"""


    vol = o3d.visualization.read_selection_polygon_volume(
        "../cropped.json")
    chair = vol.crop_point_cloud(pcd)
    o3d.visualization.draw_geometries([chair])
    print("")

    print("----- Paint chair -----")
    chair.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([chair])
    print("")
