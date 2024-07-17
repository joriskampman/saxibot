This is a logbook for the work done on the saxibot project.

---
# wednesday, 17 july 2024

---
# tuesday, 16 july 2024

## video #3: *Driving your virtual robot!*

### (separate running) robot state publisher
to start the robot state publisher (rsp)
`ros2 launch saxibot rsp.launch.py [use_sim_time:=true]`
- `saxibot` is the package in which the launch file `rps.launch.py` is found

-- new terminal --
start gazebo:
`ros2 launch gazebo_ros gazebo.launch.py`
- `gazebo_ros` is again the package in which the gazebo launch file is found
- 
-- new terminal --
spawn the saxibot:
`ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity saxibot`
`-topic robot_description` gives the topic where the robot is found
`-entity saxibot` is the name of the object model


### integrated launcher
make a launcher `sim_launcher.launch.py` which include rsp.launch.py **and** runs gazebo **and** 
runs the spawner script in 1 go
to include other launch file:

```
package_name = 'saxibot'  # must be set correctly, can be a global maybe??
incfile = os.path.join(get_package_share_directory(package_name), 'launch', 'rps.launch.py')
rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource([incfile]),
                               launch_arguments={'use_sim_time': 'true'}.items())
```

to run the spawner (which is **not** a launcher:
```
spawn_entity = Node(package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                               '-entity', 'saxibot'],
                    output='screen')
```

### moving it around
picture:
![alt text](image-1.png)

- command velocity is in topic */cmd_vel* and the type is *twist*
  - vx, vy, vz, rx, ry, rz. Only vx and rz are nonzero
- gazebo uses a *control plugin* called **gazebo_ros_diff_drive** to command and measure the motor speed of the simulated robot
  - note that all these specific plugins will be replaced by **ros2_control** at a later stage. This is the default for future packages
- the robot is *spawned* from the topic */robot_description*
- gazebo is in control of the joint states and therefore **joint_state_publisher_gui** is not required anymore
  - the topic */joint_states* subscribe to the control plugin
  - control plugin broadcasts a new frame *odom*, it is the world start position.

#### control plugin (libgazebo_ros_diff_drive.so)
Now, a new **xacro** for for the gazebo plugin *libgazebo_ros_diff_drive.so* is made, which is the 
plugin for the control of the differential drive wheels. It needs parameters to be able to calculate 
the speed and rotation, so distances and diameters.

In addition, the **odometry** will be calculated and published to a frame (called *odom* here, but 
this might be anything) and the TF and wheel TF will be updated as well.

### tele-operation via keyboard
Now, the set the **tele-operation** or *teleop* via the keyboard run:
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

I assume this connects the keyboard to the */cmd_vel* topic which is taken as the input for the 
control plugin *libgazebo_ros_diff_drive.so*

Otherwise the topic /cmd_vel is dead!

> **tips**: 
> 1. use `ros2 topic list` to get the full topics available
> 2. use `ros2 topic echo <topic>` to show the messages being published
> 3. to use a configuration in rviz: `rviz2 -d src/saxibot/config/xxxx.rviz`
> 4. to use a world in gazebo: `ros2 launch saxibot <launchfile> world:=src/saxibot/worlds/xxxx.world`
>   1. note that the saxibot itself should be removed since it will spawn via the same launch script!

