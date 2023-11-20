# Nav_stack
Autonomous motions using nav stack ros1

steps to teleop robot and save unknown map 
1. roslaunch urdfe2bot12 gazebo.launch 
2. rviz
3. rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
4. rosrun gmapping slam_gmapping scan:=/scan _base_frame:=body
5. rosrun map_server map_saver 

roslaunch my_robot_name_2dnav move_base.launch - launch the move base for autonomous drive
