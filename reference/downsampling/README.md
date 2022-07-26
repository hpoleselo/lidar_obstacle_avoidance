## Introduction

- Voxel: while pixel represents one cell in a 2D grid, a voxel represents a cube in a 3D grid. Its origin come from the mixture of pixel with volume (volume + pixel = voxel). Pretty much like in pixel is the mixture of picture + element = pixel.
- Downsampling creates a 3D voxel grid, which in other words is an approximation over the input point cloud
- A filter or a Voxel Grid is created with a certain leaf size (three dimensions must be given)

## Building and Running the Application in CPP

1. Check CMake in our computer:

` $ cmake --version `

2. Create `CMakeLists.txt`

3. Create a build folder before compiling:

` $ mkdir /build `

4. Access build folder:

` $ cd /build `

5. Compile code:

`$ cmake ..`

6. Create executable in the same directory:

` $ make `

An executable named voxel_grid_downsampling will be created in the same directory, in order to run it:

` $ ./voxel_grid_sampling `

We'll see the result by the reduced number of points:

```
PointCloud before filtering: 460400 data points (x y z intensity distance sid).
PointCloud after filtering: 41049 data points (x y z intensity distance sid).

```

If we reduce the leaf size to `0.005` we'll be getting more voxels in the grid, which would be an upsample:

```
PointCloud before filtering: 460400 data points (x y z intensity distance sid).
PointCloud after filtering: 141525 data points (x y z intensity distance sid).

```

## Running in Python

In Python instead of PCL, we make use of Open3D, though it's possible to use PCL within python (pcl-ros):

` $ pip install open3d `

It already installs all visualization and dependencies tools.

` $ python3 downsample.py `

## Resources

- https://python-pcl-fork.readthedocs.io/en/rc_patches4/tutorial/filtering.html
- http://www.open3d.org/docs/0.8.0/tutorial/Basic/pointcloud.html
- http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html
- https://www.yoctopuce.com/EN/article/compiling-the-c-library-with-cmake
- Example taken from https://pointclouds.org/documentation/tutorials/voxel_grid.html