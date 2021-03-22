# psaf_steering

## Inhalt

* [Inhalt](#inhalt)
* [Übersicht](#%c3%9cbersicht)
    * [Kurzbeschreibung](#kurzbeschreibung)
    * [Topics](#topics)
    * [Message Struktur](#message-struktur)
    * [Launch Files](#launch-files)
* [Funktionalität](#funktionalitt)
    * [Carla Twist PID Control](#carla-twist-pid-control)
	* [Carla Control Physics](#carla-control-physics)
    

## Übersicht
### Kurzbeschreibung
Dieses Paket ist für die kontrollierte Steuerung (Lenkung & Beschleunigung) des Fahrzeuges zuständig. 
Beschleunigungs- und Bremsbefehle werden hierbei über einen PID-Controller eingeregelt. Zudem enthält dieses 
Paket Konfigurationsdateien für das ROS-Package *move_base*, welches über die hier enthaltenen launch-Dateien
gestartet wird.

### Topics
#### Publish
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /carla/ego_vehicle/vehicle_control_cmd | [EgoVehicleControlInfo](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ackermann_control/msg/EgoVehicleControlInfo.msg) | Carla Twist PID Control |

#### Subscribe
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
|  /carla/ego_vehicle/twist_pid | [Twist](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html) | Carla Twist PID Control |


### Launch Files
- **carla_twist_pid_control_node.launch** startet ```carla_twist_pid_control_node.py```.  Die Parameter für den PID-Regler werden aus der Datei [settings.yaml](config/settings.yaml) geladen.
- **move_base.launch** startet die *move_base* und ```carla_twist_pid_control_node.py```. Paramter für die in der *move_base* integrierten *costmap* werden den Konfigurationsdateien im config-Ordner entnommen.
- **psaf_steering_with_vehicle.launch** startet die *carla-ros-bridge* und RVIZ mit den im config-Ordner gespeicherten Einstellungen. Zudem wird ein Fahrzeug
gespawnt und die grafische Oberfläche zur manuellen Steuerung gestartet.

## Funktionalität

### move_base
Die ROS-Node [move_base](http://wiki.ros.org/move_base) wird in unserem Projekt verwendet, um sogenannte *costmaps* zu erstellen, die Informationen über Hindernisse 
in der Welt enthält. Diese Informationen werden beispielsweise vom [Local Planner](../psaf_local_planner) verwendet.

### Carla Twist PID Control
Der Carla_Twist_PID_Controller ist eine Abwandlung des offiziellen *carla-ros-bride-Pakets* [carla_ackermann_control](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ackermann_control). 
Der Originale Controller wandelt [AckermannDrive](http://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html)-Nachrichten zu [CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ackermann_control/msg/EgoVehicleControlInfo.msg)-Nachrichten um, die vom Fahrzeug in Bewegungen umgesetzt werden. Für die Beschleunigungs- und Bremsvorgänge kommt hierzu ein PID-Controller zum Einsatz. Lenkwinkelvorgaben werden ohne Regelung direkt an das Fahrzeug weitergegeben.
Die hierfür benötigten Reglerparameter werden beim Start der Node der Datei [settings.yaml](config/settings.yaml) entnommen, können aber auch nachträglich übber *dynamic_reconfigure* angepasst werden.
Für unser Projekt wurde der Controller so verändert, dass er statt AckermannDrive-Nachrichten nun [Twist](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html)-Nachrichten verwendet, die vom [Local Planner](https://github.com/ll7/psaf1/tree/develop/psaf_ros/psaf_local_planner) versendet werden.
Eine weitere Änderung ist, dass bei einer starken Differenz der Stellgröße über einem bestimmten Threshold nun vollständig die Bremse oder das Gaspedal betätigt wird. Dies ist vor allem hilfreich, um zuverlässig vor Hindernissen abzubremsen.

### Carla Control Physics
Diese Datei ist für die Funktion des Carla_Twist_PID_Controller notwendig und wurde direkt aus der offiziellen Implementierung übernommen.
