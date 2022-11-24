import open3d as o3d
import numpy as np



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


# ----- HDBSCAN Helper Functions ------

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
