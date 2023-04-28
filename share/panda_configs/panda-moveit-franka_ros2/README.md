# panda3_moveit_config
Original source: [franka_moveit_config (from franka_ros2)](https://github.com/frankaemika/franka_ros2/tree/develop/franka_moveit_config)
## Running
```bash
ros2 launch panda3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```
```
$ ros2 node list

interactive_marker_display_94297001969872
/joint_state_publisher
/move_group
/move_group_private_94129017862064
/moveit_simple_controller_manager
/robot_state_publisher
/rviz2
/rviz2_private_139632608276064
/transform_listener_impl_559c1ec396f0
/transform_listener_impl_55c33b315690
/transform_listener_impl_55c33b3d66b0
/transform_listener_impl_7efec010df00

```
