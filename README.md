The Package consists of two packages :
1. enpm702_fall2024_ros
2. group7_final

Please follow the steps to run the node:
1. Build the package: colcon build --symlink-install
2. Source the workspace
3. Run the robot Gazebo with  world eniviroment:  ros2 launch final_project final_project.launch.py
4. Run the node:  ros2 run group7_final camera_processor
