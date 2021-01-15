<img src="https://github.com/ros-planning/moveit.ros.org/blob/main/assets/logo/moveit2/moveit_logo-black.png" alt="MoveIt 2 Logo" width="300"/>

# MoveIt 2 Beta - OMPL Constrained Planning Demo
This demo includes a launch configuration for running MoveGroup and a separate demo node.
 
The MoveGroup setup can be started:
 
     ros2 launch run_ompl_constrained_planning run_move_group.launch.py
     
This allows you to start planning and executing motions:

     ros2 launch run_move_group run_move_group_interface.launch.py
     