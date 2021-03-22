# psaf_abstraction_layer

## Inhalt
* [Inhalt](#inhalt)
* [Übersicht](#%c3%9cbersicht)
  * [Beschreibung](#Beschreibung)
  * [Topics](#Topics)
* [Funktionalität](#func)
  * [Kameras](#Kameras)
  * [Auto-Steuerung und Zustände](#Auto-Steuerung-und-Zustände)

## Übersicht
### Beschreibung
Dieses ROS Paket soll eine Abstraktionsschicht zu den Sensoren des Fahrzeugs darstellen.
Dazu wird für folgende Sensoren eine Klassenrepräsentation angeboten, welche bei neuen Daten über einen Listener die Daten aktualisiert.
- CarlaCar
  - CarlaCar: Abstraktion für die Ackermann-Steuerung und der Auto Steuerungstopic
  - VehicleStatus: Abstrahiert die CarlaEgoVehicleStatus
- Sensoren:
    - RGBCamera: Abstraktion der RGB Kamera
    - DepthCamera: Abstraktion der Tiefen-Kamera
    - SegmentationCamera: Abstraktion der Semantic Segmentation Kamera von Carla
    - FusionCamera: Abstraktion der Bilder vom CameraFusionService
    - GPS: Abstraktion für den GPS-Positionsgeber
  
### Topics
#### Publish
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /carla/{role_name}/ackermann_cmd | AckermannDrive | CarlaCar |
| /carla/{role_name}/vehicle_control_cmd | CarlaEgoVehicleControl | CarlaCar |

#### Subscribe
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /carla/{role_name}/camera/rgb/{id}/image_color | Image | RGBCamera |
| /carla/{role_name}/camera/depth/{id}/image_depth | Image | DepthCamera |
| /carla/{role_name}/camera/semantic_segmentation/{id}/image_segmentation | Image | SegmentationCamera |
| /carla/{role_name}/gnss/gnss1/fix | NavSatFix | GPS |
| /carla/{role_name}/vehicle_status | CarlaEgoVehicleStatus | VehicleStatus |
| /psaf/sensors/{role_name}/fusionCamera/{camera_name}/fusion_image | [CombinedCameraImage](../psaf_messages/msg/CombinedCameraImage.msg) | FusionCamera |

## Funktionalität
### Kameras
Die Kamera-Abstraktionen empfangen den uncodierten Bild-Daten und wandeln die mit dem ROS-Modul *cv_bridge* in das 
Bildformat von OpenCV um. Daraufhin wird der registrierte Listener über das neu empfangene Bild informiert.

Bei der Tiefen-Kamera findet zudem eine Umwandlung der relativen Position in absoluten Abstände in Meter.

### Auto-Steuerung und Zustände
Die beiden Python-Dateien *VehicleStatus* und *SegmentationCamera* beinhalten lediglich Fassaden für die Steuerung und 
die Abfrage des Status des Autos in Carla. 
