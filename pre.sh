cmds=(
      "ros2 launch livox_ros_driver2 msg_MID360_cloud_launch.py"
      "ros2 launch rm_description model.launch.py"
      "ros2 launch point_lio mapping_mid360.launch.py"
     #  "python3 ~/shaobing/src/rm_navi/rm_navigation/navi/launch/odom_to_base_node.py "

      # 当前导航障碍物链路:
      # /livox/lidar -> point_lio(/cloud_registered) ->
      # terrain_analysis(/segmentation/obstacle) ->
      # predictive_tracker(/tracked_obstacles, odom frame) ->
      # pointcloud_to_laserscan(/scan) -> Nav2
      "ros2 launch terrain_analysis terrain_analysis.launch"
      "ros2 launch predictive_tracker dynamic_tracker.launch.py"
      "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
     )
     
for cmd in "${cmds[@]}";
do
     echo Current CMD : "$cmd"
     gnome-terminal -- bash -c "cd $(pwd);source ../install/setup.bash;$cmd;exec bash;"
     sleep 0.2
done
