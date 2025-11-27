# Sensorik Auto Commands
## Nodes starten
### LIDAR starten
```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

### Arduino Serielle Kommunikation herstellen
```bash
ros2 run IArduino arduino_node
```

### Odometrie bereitstellen
```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```

### Statische Transformation bereitstellen
```bash
ros2 launch robot_tf_publisher tf_publisher_launch.py
```

### Slamtoolbox Starten
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

### Fahrzeug über Tastatur steuern
```bash
ros2 run IArduino arduino_keyboard_control.py
```

### rviz2 starten
```bash
rviz2
```

### rviz2 mit Nav2 default setup starten
```bash
ros2 run rviz2 rviz2 -d /opt/ros/kilted/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Foxglove starten
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml 
```

### Nav2 starten mit bringup
```bash
ros2 launch nav2_bringup navigation_launch.py
```

## Neue Packages anlegen
### Schritt 1:
```bash
cd ~/ros2_ws/src 
```

### Schritt 2:(Ordner und Struktur erstellen)
```bash
ros2 pkg create --build-type ament_python Name_vom_Ordner
```

### Schritt 3: Neue Datei der Node anlegen
Dort kann die selbst erstellete Python Node eingefügt werden
```bash
touch ~/ros2_ws/src/Name_vom_Ordner/Name_vom_Ordner/neue_Node.py
```
```bash
cd ros2_ws/src/Name_vom_Ordner/Name_vom_Ordner
sudo nano neue_Node.py
```

### Schritt 4: Setup Datei bearbeiten - Entry Points
Setup.py Datei öffnen
```bash
sudo nano ros2_ws/src/Name_vom_Ordner/setup.py
```
folgende Zeile in Setup hinzufügen
```bash
'neue_Node = Name_vom_Ordner.neue_Node:main',
```

### Schritt 6: Build Source
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Schritt 7: Node Starten
```bash
ros2 run Name_vom_Ordner neue_Node
```
