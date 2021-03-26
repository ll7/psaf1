# psaf_steering

## Inhalt

* [Inhalt](#inhalt)
* [Übersicht](#%c3%9cbersicht)
    * [Kurzbeschreibung](#kurzbeschreibung)
    * [Topics](#topics)
    * [Message Struktur](#message-struktur)
    * [Launch Files](#launch-files)
* [Funktionalität](#funktionalitt)
    * [move_base](#move_base)
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
| /carla/{role_name}/vehicle_control_cmd | [EgoVehicleControlInfo](https://carla.readthedocs.io/en/0.9.8/ros_msgs/#egovehiclecontrolinfomsg) | Carla Twist PID Control |

#### Subscribe
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
|  /carla/{role_name}/twist_pid | [Twist](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html) | Carla Twist PID Control |


### Launch Files
- **carla_twist_pid_control_node.launch** startet ```carla_twist_pid_control_node.py```.  Die Parameter für den PID-Regler werden aus der Datei [settings.yaml](config/settings.yaml) geladen.
- **move_base.launch** startet die *move_base* und ```carla_twist_pid_control_node.py```. Paramter für die in der *move_base* integrierten *costmap* werden den Konfigurationsdateien im config-Ordner entnommen.
- **psaf_steering_with_vehicle.launch** startet die *carla-ros-bridge* und RVIZ mit den im config-Ordner gespeicherten Einstellungen. Zudem wird ein Fahrzeug
gespawnt und die grafische Oberfläche zur manuellen Steuerung gestartet.

## Funktionalität

### move_base
Die ROS-Node [move_base](http://wiki.ros.org/move_base) stellt eine Aktion zu Verfügung, die es ermöglicht, ein Ziel in einer Welt mit einem Fahrzeug zu erreichen.
Dies wird mit einem Local und Global Planner erreicht. Hierfür wurden die Pakete der [PSAF Global Planner](../psaf_global_planner) und [PSAF Local Planner](../psaf_global_planner) implementiert, welche die ursprüngliche Funktionalität der *move_base* ersetzen.  
Zudem stellt *move_base* zwei *costmaps* zur Verfügung, die Informationen über Hindernisse in der Welt enthalten. Diese Informationen werden beispielsweise vom [Local Planner](../psaf_local_planner) verwendet. Dies stellt im Grunde das Einzige dar, was in unserem Projekt noch von *move_base* verwendet wird.

### Carla Twist PID Control
Der Carla_Twist_PID_Controller ist eine Abwandlung des offiziellen *carla-ros-bride-Pakets* [carla_ackermann_control](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ackermann_control). 
Der Originale Controller wandelt [AckermannDrive](http://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html)-Nachrichten zu [CarlaEgoVehicleControl](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ackermann_control/msg/EgoVehicleControlInfo.msg)-Nachrichten um, die vom Fahrzeug in Bewegungen umgesetzt werden. Für die Beschleunigungs- und Bremsvorgänge kommt hierzu ein PID-Controller zum Einsatz. Lenkwinkelvorgaben werden ohne Regelung direkt an das Fahrzeug weitergegeben.
Die hierfür benötigten Reglerparameter werden beim Start der Node der Datei [settings.yaml](config/settings.yaml) entnommen, können aber auch nachträglich über *dynamic_reconfigure* angepasst werden.
Für unser Projekt wurde der Controller so verändert, dass er statt AckermannDrive-Nachrichten nun [Twist](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html)-Nachrichten verwendet, die vom [Local Planner](../psaf_local_planner) versendet werden.
Eine weitere Änderung ist, dass bei einer starken Differenz der Stellgröße über einem bestimmten Threshold nun vollständig die Bremse oder das Gaspedal betätigt wird. Dies ist vor allem hilfreich, um zuverlässig vor Hindernissen abzubremsen.

### Carla Control Physics
Diese Datei ist für die Funktion des Carla_Twist_PID_Controller notwendig und wurde direkt aus der offiziellen Implementierung übernommen.
