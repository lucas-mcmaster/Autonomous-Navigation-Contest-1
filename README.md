## Useful Commands:

1. Launch Gazebo (simulator)
```cpp
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
model=:lite world:=maze
```

2. Launch keyboard teleop node
```cpp
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -r cmd_vel:=/cmd_vel_stamped
```

3. Listen to velocity commands fromt subscribed topic
```cpp
ros2 topic echo /cmd_vel_stamped
```

4. Launch SLAM toolbox for mapping
```cpp
ros2 launch slam_toolbox online_sync_launch.py
```

4.1. Launch Gazebo & SLAM toolbox (Contest 1 version)
```cpp
ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py
model=:lite world:=maze slam:=true
```

5. Open RViz2 for Turtlebot SLAM visualization
```cpp
ros2 launch turtlebot4_viz view_navigation.launch.py
```

6. Save the Gmapping map from RViz2 to a directory
```cpp
ros2 run nav2_map_server map_saver_cli -f your_map_name
```

7. Compile packages
```cpp
cd ~/ros2_ws
colcon build
```

8. Source your changes to bring them into effect
```cpp
source install/setup.bash
```

9. Echo (print out) messages from the odometry topic while Gazebo is running
```cpp
ros2 topic echo /odom
```

10. Echo (print out) messages from the hazard detection topic while Gazebo is running
```cpp
ros2 topic echo /hazard_detection
```

12. Run Contest 1 code/package (make sure Gazebo is running)
```cpp
ros2 run mie443_contest1 contest1
```

13. View structure of a laser scan
```cpp
ros2 topic type <topic name> #Shows the message type inside the specified topic
ros2 interface show <message type> #Shows the message structure of the specified message type
```
