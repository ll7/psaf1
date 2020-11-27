# Carla Start: 
./CarlaUE4.sh -carla-server -windowed -ResX=320 -ResY=240 -benchmark -fps 10

# roscore starten (nicht sicher ob nötig, kann aber ned schaden ^^)
roscore

# bridge + rviz
roslaunch carla_ros_bridge carla_ros_bridge_with_rviz.launch

# Beispielfahrzeug erstellen
roslaunch carla_ego_vehicle carla_example_ego_vehicle.launch 

# Manuelle Steuerung
roslaunch carla_manual_control carla_manual_control.launch
