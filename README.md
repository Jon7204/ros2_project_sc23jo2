# ros2-project
A lab project combines what has been taught in this module, to complete a task 

## Usage
1. Setup environent as described in the COMP3631 labs
2. Open 4 terminals and ensure the following steps are completed in order
3. In terminal 1 enter: ros2 launch turtlebot3_gazebo turtlebot3_task_world_2026.launch.py
4. In terminal 2 enter: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_sc23jo2/map/map.yaml
5. In rviz click estimate 2D pose and configure correctly for the robot shown in gazebo
6. In terminal 3 enter: ros2 run ros2_project_sc23jo2 detect_rgb
7. In terminal 4 enter: ros2 run ros2_project_sc23jo2 explore