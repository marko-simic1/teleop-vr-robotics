Required installations:

ROS KINOVA KORTEX package (humble): https://github.com/Kinovarobotics/ros2_kortex/tree/humble
MoveIt2: sudo apt install ros-humble-moveit

Starting the complete system (MoveIt, haptic driver, control nodes, robot model):

Simulation: master.launch.py
Physical robot arm: master_real_world.launch.py

In both cases, after launching the launch file, start the Servo node:
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
