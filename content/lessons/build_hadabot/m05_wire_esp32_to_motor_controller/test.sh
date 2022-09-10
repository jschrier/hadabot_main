ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 "data: 1.0"
sleep 1
ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 "data: 0.0"
ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 "data: 1.0"
sleep 1
ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 "data: 0.7"
ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 "data: 0.7"
sleep 1
ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 "data: 0.0"
ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 "data: 0.9"
sleep 1
ros2 topic pub -1 /hadabot/wheel_power_right std_msgs/msg/Float32 "data: 0.0"
ros2 topic pub -1 /hadabot/wheel_power_left std_msgs/msg/Float32 "data: 0.0"

