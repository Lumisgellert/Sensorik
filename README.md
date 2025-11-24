# Sensorik Auto Commands
## Nodes starten
### LIDAR starten
'''bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py
'''

### Arduino Serielle Kommunikation herstellen
'''bash
ros2 run IArduino arduino_node
'''

### Odometrie bereitstellen
'''bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
'''

### Statische Transformation bereitstellen
'''bash
ros2 launch robot_tf_publisher tf_publisher_launch.py
'''

### Slamtoolbox Starten
'''bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
'''

### rviz2 starten
'''bash
rviz2
'''

### Foxglove starten
'''bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml 
'''

