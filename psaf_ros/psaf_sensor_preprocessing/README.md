# psaf_sensor_preprocessing

## Inhalt
* [Inhalt](#inhalt)
* [Übersicht](#%c3%9cbersicht)
  * [Beschreibung](#Beschreibung)
  * [Topics](#Topics)
* [Funktionalität](#Funktionalität)



## Übersicht
### Beschreibung
Dieses Paket stellt einen Service *CameraFusionService* bereit, welcher
aus den drei getrennten Bild-Daten der RGB-, der Tiefen- und der Segmentation-Kamera ein gesammeltes Bild zu erzeugen.


Des Weiteren wird der *SemanticLidarPreprocessor* Service bereit gestellt.
Dieser spaltet den Semantic Lidar anhand der hinterlegten Tags in zwei neue PointClouds auf, die dann getrennt im Obstacle Layer Modul verwendet werden könnnen.

### Topics
#### Publish
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /psaf/sensors/{role_name}/fusionCamera/{camera_name}/fusion_image | [CombinedCameraImage](../psaf_messages/msg/CombinedCameraImage.msg) | FusionCameraService |
| /carla/{role_name}/semantic_lidar/processed_{lidar_name}_marking/point_cloud | PointCloud2 | SemanticLidarPreprocessor |
| /carla/{role_name}/semantic_lidar/processed_{lidar_name}_clearing/point_cloud | PointCloud2 | SemanticLidarPreprocessor |


#### Subscribe
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /carla/{role_name}/camera/rgb/{id}/image_color | Image | RGBCamera |
| /carla/{role_name}/camera/depth/{id}/image_depth | Image | DepthCamera |
| /carla/{role_name}/camera/semantic_segmentation/{id}/image_segmentation | Image | SegmentationCamera |
| /carla/{role_name}/semantic_lidar/{lidar_name}/point_cloud | PointCloud2 | SemanticLidarPreprocessor |

### Launch Dateien
- *psaf_fusion_camera_service.launch*: Startet den Perception Evaluation Service zur Verarbeitung der Verkehrssituation.
  - *role_name*: Der Rollenname des Carla-Fahrzeugs um auf die Kameras zuzugreifen.
  - *camera_group*: Der Name der Kameragruppe. Die Topics der Kameras beinhalten eine id, welche den Namen Kamera darstellen.
    Die zu gruppierenden Kameras müssen den gleichen Namen tragen. Der angegebene Name entspricht auch dem Namen der Fusion-Kamera.
  - *threshold_diff*: Der Schwellwert zur Begrenzung der zeitlichen Abweichung der verknüpften Kamerabilder zueinander.

- *psaf_semantic_processing.launch*: Startet je einen Semantic Lidar Preprocessor für die beiden im Fahrzeug vorhandenen Semantic Lidar.

## Funktionalität
### CameraFusionService
Hier werden die Bilder der drei Kameras (RGB, Tiefen und Segmentation) verarbeitet. Dabei prüft der Service zyklisch für
die empfangenen Daten, ob es bereits passenden Kameradaten im Speicher gibt. Sind ausreichend Kameradaten
vorhanden, werden diese anhand ihrer Zeitstempel verknüpft und mittels der Nachricht *CombinedCameraImage* gesendet. 
Eine tierfergreifende Bearbeitung der Bilddaten findet nicht statt.

### SemanticLidarPreprocessor
Hier wird die PointCloud2 des gewünschten Semantic Lidar analysiert. 
Anhand des Tags des Semantic Lidar können mit den [Tag-Konstanten](https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera) zugeordnet werden, um welche Art Objekt es sich handelt.
Aktuell werden so alle Fahrzeuge in die *marking* PointCloud geschoben und alles weitere wie z.B. Straßen, Bäume etc. in die *clearing* PointCloud.
Dies erlaubt eine höhere Kontrolle über den Obstacle Layer in der [Config-Datei der move_base](../psaf_steering/config/costmap_common_params.yaml#L18-22).
