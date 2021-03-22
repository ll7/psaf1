# psaf_sensor_preprocessing

## Inhalt
* [Inhalt](#inhalt)
* [Übersicht](#%c3%9cbersicht)
  * [Beschreibung](#Beschreibung)
  * [Topics](#Topics)
* [Funktionalität](#func)



## Übersicht
### Beschreibung
Dieses Paket stellt einen Service *CameraFusionService* bereit, welcher
aus den drei getrennten Bild-Daten der RGB-, der Tiefen- und der Segmentation-Kamera ein gesammeltes Bild zu erzeugen.
  
### Topics
#### Publish
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
 /psaf/sensors/{role_name}/fusionCamera/{camera_name}/fusion_image | [CombinedCameraImage](../psaf_messages/msg/CombinedCameraImage.msg) | FusionCameraService |

#### Subscribe
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /carla/{role_name}/camera/rgb/{id}/image_color | Image | RGBCamera |
| /carla/{role_name}/camera/depth/{id}/image_depth | Image | DepthCamera |
| /carla/{role_name}/camera/semantic_segmentation/{id}/image_segmentation | Image | SegmentationCamera |

### Launch Dateien
- *psaf_fusion_camera_service.launch*: Startet den Perception Evaluation Service zur Verarbeitung der Verkehrssituation.
  - *role_name*: Der Rollenname des Carla-Fahrzeugs um auf die Kameras zuzugreifen.
  - *camera_group*: Der Name der Kameragruppe. Die Topics der Kameras beinhalten eine id, welche den Namen Kamera darstellen.
    Die zu gruppierenden Kameras müssen den gleichen Namen tragen. Der angegebene Name entspricht auch dem Namen der Fusion-Kamera.
  - *threshold_diff*: Der Schwellwert zur Begrenzung der zeitlichen Abweichung der verknüpften Kamerabilder zueinander.

## Funktionalität
Hier werden die Bilder der drei Kameras (RGB, Tiefen und Segmentation) und prüft zyklisch für
die empfangenen Daten, ob es bereits passenden Kameradaten im Speicher gibt. Sind ausreichend Kameradaten
vorhanden, werden diese anhand ihrer Zeitstempel verknüpft und mittels der Nachricht *CombinedCameraImage* gesendet.


