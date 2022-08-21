# Troubleshooting

Document dedicated to steps taken in order to simulate or run Livox in the physical robot.

- Using Ubuntu 20.04

## Issue 1

While building [Livox Laser Simulation](https://github.com/Livox-SDK/livox_laser_simulation) in your own catkin workspace:

``` In file included from /home/henrique/catkin_ws/src/livox_laser_simulation/src/livox_ode_multiray_shape.cpp:14:
/home/henrique/catkin_ws/src/livox_laser_simulation/include/livox_laser_simulation/livox_ode_multiray_shape.h:10:10: fatal error: ignition/math4/ignition/math.hh: No such file or directory
   10 | #include <ignition/math4/ignition/math.hh>
```

Solution: install the math 4 dependency package from Ignition:

` $ sudo apt-get install libignition-math4-* `

Also, to prevent one possible error, make sure to change C++ version from 11 to 17 in the CMakeLists.txt from livox_laser_simulation package in line 5:

`add_compile_options(-std=c++17)`

## Issue 2

`/gazebo/spawn_urdf_model service fails`

After compiling succesfully the step above, when one runs the command to launch the simulation, the gazebo simulator and RVIZ are launched but no sensor data `/scan` topic is available and by looking at the terminal we get an error related to the spawning of the URDF failing, which is not related to the URDF specifically in this case, but to the timing of the simulation and when the sensor is available in the simulation.

When checking in `rosout.log` from the logs error:

```
spawn_urdf [gazebo_interface.py:31(spawn_urdf_model_client)] [topics: /clock, /rosout] Calling service /gazebo/spawn_urdf_model
865.162000000 INFO /spawn_urdf [gazebo_interface.py:33(spawn_urdf_model_client)] [topics: /clock, /rosout] Spawn status: SpawnModel: Entity pushed to spawn queue, but spawn service timed out waiting for entity to appear in simulation under the name livox_lidar>
```

This error is related [to this Pull Request](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1024), meaning **we have to press the Start button** in Gazebo for the simulation to start to run and the entity `livox_lidar` to appear. Furthermore I have changed in the Livox launch file:

```
<arg name="paused" value="true"/>
<arg name="use_sim_time" value="true"/>
```

After that we should be able to see the data in the `/scan` topic available and publishing `sensor_msgs/PointCloud`