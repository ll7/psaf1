# psaf_global_planner

## Inhalt

* [Inhalt](#inhalt)
* [Übersicht](#bersicht)
    * [Kurzbeschreibung](#kurzbeschreibung)
    * [Topics](#topics)
    + [Message Struktur](#message-struktur)
* [Funktionalität](#funktionalitt)
    * [Map Provider](#map-provider)
    * [Path Provider](#path-provider)
        * [Planner (path_provider)](#planner-path_provider)
        * [Replanner (path_supervisor)](#replanner-path_supervisor)
        * [Map Manager (common_road_manager)](#map-manager-common_road_manager)
    * [Planning Preprocessor](#planning-preprocessor)

## Übersicht
### Kurzbeschreibung
Das Global Planner Package

### Topics

### Message Struktur
Der ausgehende globale Pfad hat die folgende Struktur: <br>

**XRoute**:
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

## Funktionalität
Die Funktionalität dieses Packages ist grundsätzlich in drei Schritte aufzuteilen. Der [Map Provider](#map-provider) ist
zusammen mit dem [Path Provider](#path-provider) für die Bereitstellung des globalen Plans zuständig. 
Der [Planning Preprocessor](#planning-preprocessor) ist für die Kommunikation mit dem Competition Manager verantwortlich
und observiert vor der Weitergabe des Pfadziels an den eigentlichen Planungsvorgang die Umgebung des Fahrzeugs. 
Letztere Information wird dann für die potenzielle Einplanung eines U-Turns im Path Provider genutzt. 

### Map Provider

<Beschreibung der Func>


### Path Provider

Die Aufgabe des Path Providers, ist wie der Name schon vermuten lässt, einen Pfad von einem Startpunkt zu einem Zielpunkt bereitzustellen. 
Diese Pfadplanung basiert auf den vom [Map Provider](#map-provider) bereitgestellten Karteninformationen.
Der globale Pfad hat dabei die bereits in der [Übersicht](#bersicht) gezeigte [Struktur](#message-struktur).
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
der CommonRoad Search Bibliothek alle möglichen Pfade (auf dem zugrunde liegenden Lanelet-Netzwerk) berechnet.
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

<Beschreibung der Func>

#### Map Manager (common_road_manager)

<Beschreibung der Func>

### Planning Preprocessor

<Beschreibung der Func>
