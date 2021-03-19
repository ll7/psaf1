# PSAF Perception

## Inhalt
* [Inhalt](#inhalt)
* [Übersicht](#Übersicht)
  * [Beschreibung](#Beschreibung)
  * [Topics](#Topics)
* [Funktionalität](#funktionalität)
  * [Detection Service](#Detection-Service)
  * [Stop Line Detector](#Stop-Line-Detector)
  * [Stop Mark Detector](#Stop-Mark-Detector)
  * [Traffic Light Detector](#Traffic-Light-Detector)
  * [Traffic Sign Detector](#Traffic-Sign-Detector)

## Übersicht
### Beschreibung
![Übersicht perception](doc/PerceptionClassification.png)
  
### Topics
#### Publish
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /psaf/perception/traffic_signs | [TrafficSignInfo](../psaf_messages/msg/TrafficSignInfo.msg) | DetectionService |
| /psaf/perception/stop_lines | [StopLineInfo](../psaf_messages/msg/StopLineInfo.msg) | DetectionService |

#### Subscribe
Die nachfolgenden Topics werden über die Abstraktionsschicht indirekt genutzt.

| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /carla/{role_name}/camera/rgb/{id}/image_color | Image | RGBCamera |
| /carla/{role_name}/camera/depth/{id}/image_depth | Image | DepthCamera |
| /carla/{role_name}/camera/semantic_segmentation/{id}/image_segmentation | Image | SegmentationCamera |
| /psaf/sensors/{role_name}/fusionCamera/{camera_name}/fusion_image | [CombinedCameraImage](../psaf_messages/msg/CombinedCameraImage.msg) | FusionCamera |

## Funktionalität
### Detection Service
Der *Detection Service* startet die entsprechenden Detektoren und sammelt die gewonnen Wahrnehmungsdaten.
Dabei konvertiert es die abstrakten detektieren Objekten anhand ihres Labels in das passende Nachrichtenformat.
Derzeit werden folgende Detektoren gestartet:
- StopLineDetector
- TrafficLightDetector

### Stop Line Detector
Die Aufgabe des Detektors ist die Erkennung der Stopp-Linien und die entsprechenden Entfernungen.
#### Ziel
Um an den richtigen Stellen bei Ampeln und Kreuzungen zum Stehen zu kommen, benötigt das Fahrzeug die Entfernung zur Haltelinie.
Dabei sollen jedoch andere Markierungen auf der Straße nicht beachtetet werden.

#### Umsetzung
Anhand der Bilder der Segmentation-Kamera werden die Markierungen der Straße gefiltert. 
Das gefilterte Bild wird daraufhin mittels eines Kantenfilters (Canny) auf Kanten untersucht.
Ein probalistischer Hough Line Detektor ermittelt auf Basis der erkannten Kanten Linie ([siehe OpenCv](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_houghlines/py_houghlines.html)).
Dann werden die gesammelten Linien anhand ihres Winkels und der Abmessungen gefiltert. 
Anhand einer empirisch ermittelten Funktion werden die y-Koordinaten in Bezug zum Bild in eine Abstandsschätzung umgewandelt.

![Erkennung der Haltelinien](doc/stop_line-detection.png)

### Stop Mark Detector
Unter Wiederverwendung des YOLO v3 Modells der Gruppe vom Wintersemester 2019 wurde eine Detektion anhand der RGB Bilder 
umgesetzt.
#### Ziel
Um Kreuzung mit einer Stopp-Regelung zu erkennen müssen die Beschriftungen am Boden mit "Stop" erkannt werden.
![Erkennung Stop](doc/stop_detector.png)
#### Umsetzung
Das trainierte Netz wird in PyTorch geladen und die nicht benötigten Klassen werden ignoriert.
Das Modell kann unter dem [Gitlab-Server](https://git.rz.uni-augsburg.de/luttkule/carla-praktikum-ws2019/-/blob/master/carla_object_recognition/yolo-obj_last.weights) der Universität heruntergeladen werden.

### Traffic Light Detector
Die Aufgabe des Detektors ist die Erkennung der Ampel und ihres aktuellen Zustands. Zudem wird die Entfernung mittels der Tiefen-Kamera gemessen.

#### Ziel
Damit sich das Fahrzeug an den Kreuzungen mit einer Ampel korrekt verhält, müssen die Zustände der Ampeln bekannt sein.
![Erkennung der Ampeln](doc/Traffic_light_detection.png)

#### Umsetzung
![Inputbild der Fusioncamera](doc/Fusion_screenshot.png)

Das Bild der FusionCamera, also der Vereinigung der drei Kamera-Bilder von RGB-, Tiefen und Segmentation-Kamera, wird zunächst zu Erkennung der Ampeln genutzt.
Auf Basis des Segmentation-Bildes werden die Objekte im Bild, welche als Ampeln markiert sind, mit *Bounding Boxes* beschrieben.

Daraufhin wird der entsprechende Ausschnitt des RGB-Bildes durch ein Klassifikationsnetzes (Resnet18) klassifiziert.
Dabei stehen folgende Klassen zur Wahl:
- **Red**: Die Ampel zeigt rot
- **Green**: Die Ampel zeigt grün
- **Yellow**: Die Ampel zeigt gelb
- **Back**: Die Rückseite der Ampel ist zu sehen.

Als Ergebnis der Klassifizierung erhält man die wahrscheinlichste Klasse und das Vertrauen des Modells in das Ergebnis.

Anschließend wird für jede Bounding Box, welche nicht als *Back* klassifiziert wurde, anhand der Segmentierungsdaten eine Maske erstellt.
Diese Maske wird genutzt die richtigen Pixel des Tiefenbildes zu wählen.
Über eine Mittelwertsberechnung wird dann anhand der Pixel die Entfernung zur Ampel bestimmt.

### Traffic Sign Detector
Die Aufgabe des Detektors ist die Erkennung der Schilder und deren Entsprechung. Zudem wird die Entfernung mittels der Tiefen-Kamera gemessen.

#### Ziel
Damit sich das Fahrzeug an den Kreuzungen mit einer Ampel korrekt verhält, müssen die Zustände der Ampeln bekannt sein.

![Erkennung der Schilder](doc/Traffic_sign_detection.png)

#### Umsetzung
Das Bild der FusionCamera ([siehe Traffic Light Detector](#Traffic-Light-Detector)) wird zunächst zu Erkennung der Schilder genutzt.
Auf Basis des Segmentation-Bildes werden die Objekte im Bild, welche als Verkehrsschilder markiert sind, mit *Bounding Boxes* beschrieben.

Daraufhin wird der entsprechende Ausschnitt des RGB-Bildes durch ein Klassifikationsnetzes (Resnet18) klassifiziert.
Dabei stehen folgende Klassen zur Wahl:
- **back**: Die Rückseite eines Schilder
- **speed_30**: Ein europäisches Geschwindigkeitsbeschränkungsschild auf 30km/h
- **speed_60**: Ein europäisches Geschwindigkeitsbeschränkungsschild auf 60km/h
- **speed_90**: Ein europäisches Geschwindigkeitsbeschränkungsschild auf 90km/h
- **speed_limit_30**: Ein amerikanisches Geschwindigkeitsbeschränkungsschild auf 30mp/h
- **speed_limit_40**: Ein amerikanisches Geschwindigkeitsbeschränkungsschild auf 40mp/h
- **speed_limit_60**: Ein amerikanisches Geschwindigkeitsbeschränkungsschild auf 60mp/h
- **stop**: Ein Stopp-Schild

Als Ergebnis der Klassifizierung erhält man die wahrscheinlichste Klasse und das Vertrauen des Modells in das Ergebnis.

Anschließend wird für jede Bounding Box, welche nicht als *Back* klassifiziert wurde, anhand der Segmentierungsdaten eine Maske erstellt.
Diese Maske wird genutzt die richtigen Pixel des Tiefenbildes zu wählen.
Über eine Mittelwertsberechnung wird dann anhand der Pixel die Entfernung zur Ampel bestimmt.
