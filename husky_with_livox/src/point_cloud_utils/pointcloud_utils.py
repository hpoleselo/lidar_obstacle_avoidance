import numpy as np

def ros_to_numpy(ros_pointcloud_msg):
    """
    Converts sensor_msgs/PointCloud.msg to numpy array.
    http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud.html
    """
    xyz = []

    print(ros_pointcloud_msg.points[0])

    for data in ros_pointcloud_msg.points:
        xyz.append([data.x, data.y, data.z])
        #rgb = np.append(rgb,[[r,g,b]], axis = 0)	

    return np.array(xyz)

def downsample():
    """
    Converts numpy array to Open3D class and downsamples it.
    """
    pass
