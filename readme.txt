Potrebno instalirati:
	ROS KINOVA KORTEX paket(humble): https://github.com/Kinovarobotics/ros2_kortex/tree/humble
	Moveit2: sudo apt install ros-humble-moveit

Pokretanje cijelog sustava(Moveit, Haptic driver, čvorovi za upravljanje, model robota)
	simulacija: master.launch.py
	fizička ruka: master_real_world.launch.py
	U oba slučaja nakon pokretanja launch file-a pokrenuti Servo čvor: ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
