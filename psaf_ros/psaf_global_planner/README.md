# psaf_global_planner

## Inhalt

* [Inhalt](#inhalt)
* [Übersicht](#%c3%9cbersicht)
    * [Kurzbeschreibung](#kurzbeschreibung)
    * [Topics](#topics)
    * [Message Struktur](#message-struktur)
    * [Launch Files](#launch-files)
* [Funktionalität](#funktionalitt)
    * [Map Provider](#map-provider)
        * [Map Converter](#map-converter-map_provider)
        * [Landmark Provider](#landmark-provider-landmark_provider)
        * [Map Supervisor](#map-supervisor-map_supervisor)
    * [Path Provider](#path-provider)
        * [Planner (path_provider)](#planner-path_provider)
        * [Replanner (path_supervisor)](#replanner-path_supervisor)
        * [Map Manager (common_road_manager)](#map-manager-common_road_manager)
    * [Planning Preprocessor](#planning-preprocessor)    
    * [Rviz Plugin](#rviz-plugin)

## Übersicht
### Kurzbeschreibung
Die Hauptaufgabe des Global Planner Package ist die der initialen Planung und der dynamischen Neuplanung.
Als Ergebnis wird ein globaler Plan bereitgestellt. Zur Berechnung von diesem müssen entsprechende Karteninformationen
generiert, bzw. konvertiert und aufbereitet werden. Im Verlauf des Projekts wurde der Umfang des globalen Plans 
und damit die geforderte Funktionalität erweitert.
Der globale Plan beinhaltet nun deutlich mehr Informationen als eine simple Abfolge von XY-Koordinaten.
Dazu lässt sich auf den Message Typ des gepublishten Plans, der [XRoute](#message-struktur), verweisen. 

### Topics
#### Publish
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /psaf/status | String | Path Provider |
| /psaf/xroute | [XRoute](../psaf_messages/msg/XRoute.msg) | Path Provider|

#### Subscribe
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /psaf/goal/set_instruction | [PlanningInstruction](../psaf_messages/msg/PlanningInstruction.msg) | Path Provider |
| /psaf/planning/obstacle | [Obstacle](../psaf_messages/msg/Obstacle.msg) | Path Provider|
| /carla/world_info | CarlaWorldInfo | Map Provider |


### Message Struktur

**XRoute**:
```
- uint32 id
- bool isUTurn
- XLanelet[] route:
    - bool hasLight
    - bool isAtIntersection
    - bool hasStop
    - bool isLaneChange 
    - uint32 id
    - CenterLineExtended[] route_portion:
        - float32 x
        - float32 y
        - float32 z
        - uint8 speed
        - float32 duration
        - float32 distance
```
    
**Obstacle**:
```
- uint8 id
- geometry_msgs/Point[] obstacles
```

**PlanningInstruction**:
```
- geometry_msgs/Pose goalPoint
- bool planUTurn
- float32 obstacleDistanceForward
- float32 obstacleDistanceLeft
```

### Launch Files
- **psaf_global_planner_path_common_roads.launch** startet ```path_supervisor_common_roads.py``` 
  und ```planning_preprocessor.py```. Dies beinhaltet, unter Verwendung der CommonRoads Bibliothek, alle in 
  diesem Package beschriebenen Funktionalitäten. Zur Generierung des o.g. globalen Plans muss dieses launch file
  ausgeführt werden. Über Parameter kann der das Verhalten des Planners gesteuert werden. Die Beachtung 
  von Verkehrsregeln, Debugging Nachrichten, der Export der XRoute in eine DebugPath Datei 
  (für den [Scenario Runner](../psaf_scenario/README.md)) und das 
  Erzwingen eines U-Turns (wenn mgl) kann aktiviert werden.

- **psaf_global_planner_path_lanelet2.launch** startet den ```path_provider_lanelet2.py```, welcher auf Basis des Lanelet2
  Frameworks einen einfachen globalen Plan generiert. Dieses launch file sollte nur zu Testzwecken der Fähigkeiten von
  Lanelet2 ausgeführt werden.


## Funktionalität
Die Funktionalität dieses Packages ist grundsätzlich in drei Schritte aufzuteilen. Der [Map Provider](#map-provider) ist
zusammen mit dem [Path Provider](#path-provider) für die Bereitstellung des globalen Plans zuständig. 
Der [Planning Preprocessor](#planning-preprocessor) ist für die Kommunikation mit dem Competition Manager verantwortlich
und observiert vor der Weitergabe des Pfadziels an den eigentlichen Planungsvorgang die Umgebung des Fahrzeugs. 
Letztere Information wird dann für die potenzielle Einplanung eines U-Turns im Path Provider genutzt. 

### Map Provider

Der Map Provider ist für die Bereitstellung der Karteninformationen verantwortlich. Der Kern dieser Informationen
bildet dabei das Lanelet-Netzwerk. Eine Lanelet ist eine als Straßenabschnitt zu bezeichnende Darstellung, definiert
durch die Grenzen der Fahrspur und ihrer Mittellinie. Das Lanelet-Netzwerk ergibt sich somit als Graph aus 
Straßenabschnitten, wobei deren Verbindungen durch Kanten (Vor- und Nachfolger- Referenzen) repräsentiert sind.
Eine detailliertere Beschreibungen folgt in den Unterkapiteln.

#### Map Converter (map_provider)
Der Map Converter bildet die Basis für den [Map Supervisor](#map-supervisor-map_supervisor). In Ihm wird die Karte im 
OpenDrive Format geladen. In einem zweiten Schritt wird die Karte in ein Common Road Scenario konvertiert, welches die Basis 
für den [Path Provider](#path-provider) darstellt.

Zusätzlich bietet er noch die Möglichkeiten die OpenDrive Karte als OpenStreetMap oder als Lanelet2 Karte zu exportieren.

#### Landmark Provider (landmark_provider)
Der Landmark Provider liest alle Ampel und Schilder Information aus der Carla API aus und stellt sie dem 
[Map Supervisor](#map-supervisor-map_supervisor) als strukturierte Information zur Verfügung. 
Dieser reichert damit das Common Road Scenario an. 

#### Map Supervisor (map_supervisor)
Da der [Map Converter](#map-converter-map_provider) nur eine "leere" Karte generiert, also eine Karte ohne 
Ampel und Schilder Informationen, erweitert der Map Supervisor seine Basisklasse um Algorithmen welche die Informationen, 
die vom [Landmark Provider](#landmark-provider-landmark_provider) bereitgestellt werden, in die Karte einfügen.
Hierbei werden folgende Bereiche unterschieden:
- **Schilder allgemein** (z.B Stoppschilder, Geschwindigkeitsschilder) werden der Lanelet hinzugefügt, welche sich
  positionstechnisch am nächsten am Schild befinden und mit deren Orientierung des Schildes übereinstimmt.
- **Kreuzungen** werden daran erkannt, dass der Nachfolger einer Lanelet mehrere Vorgänger besitzt. Genau dann qualifiziert 
  sich diese Lanelet als Bestandteil einer Kreuzung. Um sämtliche Lanelets auf einer Kreuzung zu erkennen wird 
  anschließend für jede Lanelet folgende Metrik angewandt.
  Ist mein (rechter oder linker) Nachbar Teil einer Kreuzung, dann bin ich es auch. Lanelets welche nicht die
  vordefinierte Nachbareigenschaft erfüllen, aber trotzdem nebeneinander liegen werden über ihre geografische Position
  ebenso richtig zugeordnet.
- **Ampeln** werden immer am Ende einer Lanelet positioniert. Eine Metrik zur Wahl jener Lanelet, für die die Ampel gilt
  und die sowohl für europäische als auch amerikanische Ampelsysteme funktioniert ist die folgende.
  Suche nach derjenigen Lanelet, deren Orientierung entsprechend zur Ampel ausgerichtet ist und 
  welche sich am nähesten an der Ampel befindet, wobei der Anfang dieser Lanelet allerdings weiter von der Ampel
  entfernt sein muss als ihr Ende. Dadurch wird im amerikanischen System die Ampel nicht fälschlicherweise auf eine Lanelet
  hinter der Kreuzung gesetzt.
  Ebenso wird allen (linken und rechten) Nachbarn der gefundenen Lanelet diese Ampel zugeordnet.
- **Stopplinien** werden zu Stoppschildern umgewandelt. Diese werden an das Ende der Lanelet gesetzt, welche den Stop beinhalten soll.
  Die zutreffende Lanelet ist die nächstgelegene Lanelet, die sich **vor** und **nicht in** einer Kreuzung befindet
  und deren Orientierung zum Stoppschild ausgerichtet ist.


### Path Provider

Die Aufgabe des Path Providers, ist wie der Name schon vermuten lässt, einen Pfad von einem Startpunkt zu einem Zielpunkt bereitzustellen. 
Diese Pfadplanung basiert auf den vom [Map Provider](#map-provider) bereitgestellten Karteninformationen.
Der globale Pfad hat dabei die bereits in der [Übersicht](#bersicht) gezeigte [Struktur](#message-struktur) der (e)X(tended)Route.
Der Path Provider Lanelet2 befindet sich zu Zwecken der Veranschaulichung der Möglichkeiten der Lanelet2 Bibliothek 
ebenso in diesem Package.
Wenngleich die Kompatibilität (bei Starten des entsprechenden launch file in /psaf_starter) dazu gewährleistet wird, 
sind erweiterte Funktionen, wie des Replannings, der Anpassung der Karte, sowie der Bereitstellung 
des erweiterten gezeigten globalen Pfades, im Path Provider Lanelet2 nicht enthalten. 
Grund hierfür ist, dass diese Bibliothek in Kombination mit der dazu 
benötigten Kartenkonvertierung nicht überzeugen konnte. Daher wurde sich sowohl im weiteren Verlauf des Projekts, als 
auch hier in der Dokumentation auf die Implementierung beschränkt, die die CommonRoads Bibliothek verwendet.

Die grundsätzliche Funktionsweise dieses Abschnitts lässt sich, zur Wahrung der Übersicht, in drei Teile
mit klaren Aufgabenbereichen untergliedern. Die des [Planners](#planner-path_provider), des [Replanners](#replanner-path_supervisor)
und des [Map Managers](#map-manager-common_road_manager). Eine genauere Erläuterung erfolgt in den jeweiligen Abschnitten.

#### Planner (path_provider)

Die Basis Klasse, jene von der der [Replanner](#replanner-path_supervisor) erbt, ist der PathProviderCommonRoads.
Der Ablauf eines Pfad-Planungs-Aufrufs lässt sich wie folgt beschreiben. Zunächst wird die aktuelle Position des Fahrzeugs,
gemessen mithilfe des GPS Sensors, festgelegt. Die Planungs-Aufforderung beinhaltet die geforderte Startposition. Aus 
den genannten Informationen wird anschließend ein Planungsproblem formuliert. Auf dessen Grundlage werden dann mithilfe
der CommonRoad Search Bibliothek alle möglichen Pfade (auf dem zugrunde liegenden Lanelet-Netzwerk) berechnet (A-Star).
Anschließend wird aus den im [CommonRoadManager](#map-manager-common_road_manager) vorgehaltenen Message Informationen, 
die Pfad Message aus den zu berücksichtigen Lanelets (bzw. Lanelet Abschnitten) zusammengesetzt.
Aus dieser Kandidatenliste wird darauffolgend der (Pfad-) Kandidat ausgewählt, welcher die geringsten Pfad-Kosten aufweist.
Für den Planungsfall mit Verkehrsregeln wird dazu für jeden Pfad (mithilfe der Pfad-Message) die Zeitdauer berechnet. Zu Beachten ist dabei, dass 
für jedes auf dem jeweiligen Pfad vorkommende Stoppschild beziehungsweise jede Ampel eine (im launch file) definierte
Penalty addiert wird. Spurwechsel werden gleichermaßen durch die Addition eines kleinen Zeitwertes „bestraft". Die Höhe
dieser „Strafen" ist dabei frei wählbar. Entsprechend sind Strafen durch die Wahl des Wertes 0 auch deaktivierbar.
Der schnellste Pfad ist im Sinne der Aufgabenstellung der Gesuchte und so wird die Message für diesen Pfad zurückgegeben.
Für den Planungsfall ohne Verkehrsregeln wird hier anstelle der Zeitdauer eines Pfades die Gesamtdistanz eines Pfades 
berücksichtigt und in Konsequenz der kürzeste Pfad zurückgegeben. Grund hierfür ist, dass bei der Fahrt ohne 
Verkehrsregeln Geschwindigkeitslimits ignoriert werden und auch an Ampeln nicht auf Grünphasen gewartet wird. 

Zusätzlich wird in Abhängigkeit der übergebenen Informationen des [Planning Preprocessors](#planning-preprocessor)
ein initialer U-Turn eingeplant. Dazu wird zunächst betrachtet, ob ausreichend Platz zur Verfügung steht. Das heißt, ob
die entgegen der aktuellen Richtung verlaufende Fahrspur erreichbar ist. Ist dies der Fall wird zusätzlich zur 
eigentlichen Planung ebenso eine Planung von der gegenüberliegenden Seite durchgeführt. Die aus den beiden Planungen
(mit oder ohne U-Turn) resultierenden Pfadkosten werden verglichen, wobei für den U-Turn eine optionale Penalty addiert
wird. Die Message des Pfads mit den geringeren Kosten wird zurückgegeben.
Kurz gesagt wird also, falls ein U-Turn berücksichtigt werden soll, genau dann ein U-Turn ausgeführt, wenn dieser von 
Vorteil ist.

#### Replanner (path_supervisor)

Der Path Supervisor erweitert die Funktionalität des Planners um die Möglichkeit dynamisch, während der Laufzeit, eine
Neuplanung auf Basis von erkannten Hindernissen anzustoßen. Eine Neuplanung wird initiiert, wenn der Provider eine 
Obstacle Nachricht über den Subscriber erhält.

Nach Erhalt der Nachricht wir in einem ersten Schritt überprüft, ob die erhaltenen Hindernissen relevant für den 
derzeitigen Fahrtverlauf sind. Konkret werden nur Hindernisse beachtet, die sich vor und neben dem Fahrzeug befinden,
wobei die Fahrtrichtung der Straße, auf der ein Hindernis eingeplant wird, ebenfalls eine Rolle spielt. 
Dementsprechend werden nur Hindernisse, die in Fahrtrichtung unseres Fahrzeugs liegen, eingefügt.

Für die relevanten Hindernisse wird im zweiten Schritt überprüft, ob sich das Hindernis auf der gleichen Lanelet 
wie das Auto befindet.
- Falls sich das Hindernis nicht auf der Lanelet des Autos befindet, wird es nur seiner eigenen Lanelet hinzufügt.
- Falls sich das Hindernis auf der Lanelet des Autos befindet, wird die Lanelet in drei Abschnitte aufgeteilt und das 
  Hindernis wird auf den dritten Abschnitt eingefügt. 
  Das genaue Vorgehen eines Teilungsprozesses wird im [CommonRoadManager](#map-manager-common_road_manager) beschrieben.
  - Abschnitt eins ist der Teil der Lanelet auf der sich das Auto befindet. **[Lanelet Start, Position Auto]**
  - Abschnitt zwei ist der Teil der Lanelet auf der sich das zwischen Auto und dem Hindernis befindet. 
    Dieser Abschnitt wird benötigt damit der Überholvorgang sauber zwischen Auto und Hindernis eingeplant werden kann. 
    **]Position Auto, Position Hindernis[**
  - Abschnitt drei ist der Abschnitt der Lanelet auf der sich das Hindernis befindet. **[Position Hindernis, Lanelet Ende]**
    
Zu erwähnen ist, dass das Teilen einer Lanelet unmittelbar ein Aktualisieren aller Referenzen der Lanelets in der Umgebung, sowie ein
Teilen aller adjazenten Lanelets zur Folge haben muss, da nur so das Lanelet-Netzwerk konsistent und mögliche 
Routenplanungen (inkl. Spurwechseln) plausibel gehalten werden können. Das fällt unter den Aufgabenbereich des 
[Map Managers](#map-manager-common_road_manager) und wird in diesem Kapitel genauer beschrieben.

Im dritten und letzten Schritt wird die Neuplanung angestoßen. Hierbei gilt es nur zu beachten, 
dass Straßen mit einem Hindernis ein hohes Kantengewicht zugeteilt bekommen, sodass der Planungsalgorithmus
Straßen mit Hindernissen nur dann wählt, wenn es keine Alternativen gibt. Also beispielsweise, wenn sich vor und neben dem 
Fahrzeug ein anderes Fahrzeug befindet. Ist dies der Fall, fordert der globale Plan folglich indirekt dazu auf dem 
vorausfahrenden Fahrzeug zu folgen, da es sich trotz Hindernis weiterhin um die, je nach Metrik, optimalste Route handelt.

Ein weiterer wichtiger Punkt ist es, dass eine Neuplanung immer auf den Originalkartendaten, 
also auf Kartendaten ohne Hindernisse und ohne modifiziertes Lanelet-Netzwerk ausgeführt wird.

#### Map Manager (common_road_manager)

Die Idee des Common Road Managers ist es, dass er die Hauptschnittstelle zwischen der Planung und den Common Road 
Kartendaten abbildet. Er besitzt hierfür zwei grundlegende Funktionalitäten:
1. Er berechnet die vorgehaltenen Informationen, welche für die Planung im [Replanner](#replanner-path_supervisor) benötigt werden.
   Konkret handelt es sich hierbei um das Common Road Scenario und eine [XLanelet](#message-struktur) Repräsentation jeder Lanelet.
   Letztere ist als vorgefertigte Nachricht jeder Lanelet anzusehen, sodass im [Planner](#planner-path_provider)
   nur noch diese Nachrichten (gesamt, oder in Teilen bei Spurwechseln) für alle Lanelets eines Pfades zusammengesetzt werden müssen.
   Diese Informationen werden im Konstruktor des Managers einmalig (bzw. einmalig für jede Lanelet) berechnet. 
   Im späteren Verlauf werden nur noch Änderungen, die zum Beispiel durch Lanelet-Splits (siehe Abschnitt 2.) hervorgerufen werden, vorgenommen.
   Das führt zu einer effizienten Reaktion Änderungswünsche und hat den Vorteil, dass die sehr umfangreichen Inhalte 
   der XRoute nicht in jedem Planungsaufruf neu generiert werden müssen.
   
2. Zusätzlich ist er für das Aufsplitten von Lanelets verantwortlich. Hierbei werden die angegebene Lanelet und alle 
   bekannten rechten und linken Nachbarn am angegebenen Punkt aufgetrennt. Um eine Lanelet aufzutrennen wird diese 
   erst entfernt, um anschließend durch zwei Neue ersetzt zu werden. Im Zuge dessen werden die Referenzen im 
   gesamten Straßennetz aktualisiert, um die Integrität des Netzes beizubehalten. Das Referenzenupdate wird ebenfalls 
   aus Geschwindigkeitsgründen anhand einer vorher berechneten Nachbarschafts-Information vollzogen. 
   
### Planning Preprocessor

Der Planning Preprocessor steht im Zeitlichen Ablauf unserer Implementierung ganz am 
Anfang. Er stellt die Schnittstelle zwischen dem vorgegebenem [Competition Manager](https://github.com/ll7/psaf20/tree/main/psaf20_competition_manager) und 
unserem Projekt dar.
 
Für den Wettbewerb wird der Start- und Zielpunkt der aktuellen Fahrt durch den 
[Competition Manager](https://github.com/ll7/psaf20/tree/main/psaf20_competition_manager) vorgegeben. Für diesen Fall achtet der Planning Preprocessor 
darauf, ob das Fahrzeugt mit dem [set_position_client](https://github.com/ll7/psaf20/blob/main/psaf20_competition_manager/src/set_position_client.py) des Competition Managers neu 
gespawnt wurde. Tritt dies ein, liest der Preprocessor die Zielkoordinaten aus dem 
*ROS-Parameter-Server* und gibt sie mit einer [PlanningInstruction](#message-struktur)-Message an den 
[Global Planner](#path-provider) weiter. 
 
Bei Bedarf prüft der Prepocessor noch wie viel Platz neben dem Fahrzeug für einen 
U-Turn zur Verfügung steht und fügt diese Information der ausgehenden Nachricht hinzu. Die Erkennung des freien 
Bereichs links/vorne neben dem Auto funktioniert dabei analog zum semantic_lidar_processor mit den Daten von zwei
LIDAR-Sensoren, die auf Hindernisse wie Gebäude, Mauern und Zäune regieren. 

![uturn-lidar](https://github.com/ll7/psaf1/tree/develop/psaf_ros/psaf_global_planner/doc/uturn_lidar.png)
 
Ob ein U-Turn eingeplant wird, hängt von den *rosparam*-Parametern **obeyRules** und 
**alwaysUTurn** ab. Diese Parameter werden wie in [psaf_starter](https://github.com/ll7/psaf1/tree/develop/psaf_ros/psaf_steering) beschrieben, über 
verschiedene launch-Dateien voreingestellt.
 
Für einen Betrieb ohne den Competition Manager steht ein RVIZ-Panel zu Verfügung, 
mit dem ein Zielpunkt manuell vorgegeben werden kann. Als Startpunkt wird dann die 
aktuelle Position des Fahrzeuges verwendet.


### Rviz Plugin

Das Rviz Plugin stellt ein Panel in Rviz bereit, über das die XYZ Zielposition für die Navigationsaufgabe des Fahrzeugs
gesetzt werden kann. Außerdem wird mittels einer Statusanzeige über den aktuellen Zustand der Planung
informiert.
 
![rviz-panel](https://github.com/ll7/psaf1/tree/develop/psaf_ros/psaf_global_planner/doc/rviz-panel.png)
