# Husky Customization with Livox LiDAR

I had several issues while trying to add a custom sensor to Husky, mainly because the documentation provided [in here](https://www.clearpathrobotics.com/assets/guides/kinetic/husky/CustomizeHuskyConfig.html) wasn't clear and I was getting cyclic imports (that's my assumption at least).
So I came up with a troubleshooting and a manual approach to add a sensor to Husky, which is not so elegant and I'll be trying to make it more elegant in the future.

## Table of Contents

- [Husky Customization with Livox LiDAR](#husky-customization-with-livox-lidar)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
  - [Procedure to Run Customized Husky with Livox](#procedure-to-run-customized-husky-with-livox)
  - [Working with Offline Point Clouds](#working-with-offline-point-clouds)
  - [Procedure to Customize Husky with New Sensor](#procedure-to-customize-husky-with-new-sensor)
  - [Running](#running)


## Installation

Clone this repository to your local workspace (assuming it's catkin_ws):

` $ git clone https://github.com/hpoleselo/lidar_obstacle_avoidance `

Make sure to install all necessary dependencies or just run the shell file located in `/shell/install_dependencies.sh` in this repository, first install teleoperation twist keyboard to control Husky:

` $ sudo apt-get install ros-noetic-teleop-twist-keyboard `

Then all packages related to Husky:

```shell
$ sudo apt-get install ros-noetic-husky-gazebo
$ sudo apt-get install ros-noetic-husky-control
$ sudo apt-get install ros-noetic-husky-description
$ sudo apt-get install ros-noetic-husky-msgs
$ sudo apt-get install ros-noetic-husky-simulator
$ sudo apt-get install ros-noetic-husky-viz
```

Clone Livox repository to your catkin_workspace:

`$ git clone git@github.com:Livox-SDK/livox_laser_simulation.git`

## Procedure to Run Customized Husky with Livox

Make sure to follow the steps given in Installation section and then:

1. Update your Livox URDF with the new adaption by: 

```shell
$ roscd husky_with_livox/urdf
$ cp livox_mid70.xacro ~/catkin_ws/src/livox_laser_simulation/urdf/
```

2. Now update Husky launcher to point to our customized xacro:

```shell
$ roscd husky_with_livox/launch
$ sudo cp description.launch /opt/ros/noetic/share/husky_description/launch
```

**Note**: Will find more elegant solution for this, instead of using sudo to copy a file, reference the Husky description somehow by a custom launch file.

3. Test it by running the launch file:

```shell
$ roslaunch husky_with_livox husky_with_livox.launch
```

## Working with Offline Point Clouds

Livox Simulation Laser is using an old type of PointCloud message, which is simpler (just requires XYZ), but in the other hand most of the tools used for PCLs are based on PointCloud2, meaning the conversion from PointCloud to PointCloud2 is needed in order to, for instance, generate a .PCD file from a rosbag containing PointClouds.
To convert from PointCloud to PointCloud2, clone [this repository](https://github.com/pal-robotics-forks/point_cloud_converter) to your catkin workspace:

``` $ git clone git@github.com:pal-robotics-forks/point_cloud_converter.git ```

catkin_make your workspace and then customize your launch file like this (such file is present in this repository):

```xml
<launch>
	<node pkg="point_cloud_converter" name="point_cloud_converter" type="point_cloud_converter_node" >
		<remap from="points_in" to="/scan"/>
		<remap from="points2_out" to="/converted_point_cloud_2" />
	</node>
</launch>
```

We're basically taking as input our topic that publishes PointCloud and converting it to an output topic `/converted_point_cloud_2`.

First make sure to run the simulation you want to record to a bag, in this specific case we were using our own custom world and Husky with livox:

` $ roslaunch husky_with_livox husky_with_livox.launch`

And run our converter node:

``` $ roslaunch point_cloud_converter point_cloud_converter.launch ```

We can check that the conversion is being made by checking:

``` $ rostopic echo /converted_point_cloud_2 ```

Record the simulation:

` $ rosbag record -a`

Then play your rosbag file (one is provided in this repository):

``` $ rosbag play test_bag.bag ```

Now to convert it to a PCD file:

``` $ rosrun pcl_ros bag_to_pcd table_chair.bag /converted_point_cloud_2 .```

Expected Output:

```shell
Data saved to ./160.400000000.pcd
Got 1000 data points in frame laser_livoxx on topic /converted_point_cloud_2 with the following fields: x y z
Data saved to ./160.500000000.pcd
Got 1000 data points in frame laser_livoxx on topic /converted_point_cloud_2 with the following fields: x y z
Data saved to ./160.600000000.pcd
```

Note that each message will be an array of points - in this specific case - containing 1k points per message, since the frequency of publishing is 10Hz we get around 10 messages per seconds which leads to 10k points per second and each message will result in one pcd file.

## Procedure to Customize Husky with New Sensor

**Note that those files are already present in this folder so there's no need to follow all of them, just step 5.**.

1. Create a custom package in your catkin workspace with a `/urdf` folder
2. Copy the whole xacro file from the original `husky_description/urdf/husky-urdf.xacro` file to a `custom_husky.urdf` in your package.
3. Add the lines you want to incorporate/modify to your robot:
    
    ```xml
    <link name="bowler_link">
        <visual>
          <geometry>
            <mesh filename="package://husky_with_livox/meshes/red_bowler.dae" />
          </geometry>
        </visual>
        <collision>
          <origin xyz="0 0 0.07" rpy="0 0 0" />
          <geometry>
            <box size="0.32 0.25 0.14" rpy="0 0 0"/>
          </geometry>
        </collision>
      </link>
    
      <joint name="bowler_joint" type="fixed">
        <parent link="top_plate_link" />
        <child link="bowler_link" />
        <origin xyz="0.1 -0.1 0" rpy="0 0 0.5" />
    </joint>
    ```
    
4. Inside your custom urdf directory, add an `empty.urdf` that contains:
    
    ```xml
    <robot>
      <!-- This file is a placeholder which is included by default from
           husky.urdf.xacro. If a robot is being customized and requires
           additional URDF, set the HUSKY_URDF_EXTRAS environment variable
           to the full path of the file you would like included. -->
    </robot>
    ```
    
5. Do a `roscd husky_description/launch` and then `sudo gedit description.launch` make sure to point to our newly created xacro file:
    
    ```xml
    <!-- Change from: -->
      <param name="robot_description" command="$(find xacro)/xacro '$(find husky_description)/urdf/husky.urdf.xacro'
        robot_namespace:=$(arg robot_namespace)" />
    
    <!-- To: -->
    <param name="robot_description" command="$(find xacro)/xacro '$(find husky_with_livox)/urdf/custom_description.urdf.xacro'
      robot_namespace:=$(arg robot_namespace)" />
    ```

## Running

To test if it's working first run the raw husky environment:

`$ roslaunch husky_gazebo husky_empty_world.launch`

Since we have modified the original xacro from `husky_description`, we should see the livox mounted on it and generating a scan in Gazebo:

![husky_com_livox](https://user-images.githubusercontent.com/24254286/197667896-83a5a4aa-c571-412b-a818-82c0da7d1422.png)

Then control the robot by using:

`$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

We should be able to check for the incoming messages containing the point cloud:

`$ rostopic echo /scan`

```shell
  - 
    x: 0.24610918760299683
    y: 0.006262338254600763
    z: -0.09895515441894531
  - 
    x: 0.2433687448501587
    y: 0.006100865080952644
    z: -0.09868306666612625
```

And even measuring the frequency of publication of this topic by:

`$ rostopic hz /scan`

```shell
subscribed to [/scan]
WARNING: may be using simulated time
average rate: 10.381
	min: 0.081s max: 0.105s std dev: 0.01087s window: 4
average rate: 10.152
	min: 0.081s max: 0.105s std dev: 0.00766s window: 9
average rate: 10.059
```