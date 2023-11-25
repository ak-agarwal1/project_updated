# Start RVIZ
  
ros2 run rviz2 rviz2 -d viewatlas.rviz

# Start Joint State Publisher

ros2 run robot_state_publisher robot_state_publisher atlas_v5.urdf

# Start Trajectory

ros2 run project_updated project_main
