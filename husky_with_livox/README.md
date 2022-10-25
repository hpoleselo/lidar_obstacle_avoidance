# Husky Customization with Livox LiDAR

I had several issues while trying to add a custom sensor to Husky, mainly because the documentation provided [in here](https://www.clearpathrobotics.com/assets/guides/kinetic/husky/CustomizeHuskyConfig.html) wasn't clear and I was getting cyclic imports (that's my assumption at least).
So I came up with a troubleshooting and a manual approach to add a sensor to Husky, which is not so elegant and I'll be trying to make it more elegant in the future.

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

## Procedure to Customize

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