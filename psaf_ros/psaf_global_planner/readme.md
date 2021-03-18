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

<Beschreibung der Func>

Die Aufgabe des Path Providers, ist wie der Name schon vermuten lässt, einen Pfad von einem Startpunkt zu einem Zielpunkt bereitzustellen. 
Diese Pfadplanung basiert auf den vom [Map Provider](#map-provider) bereitgestellten Karteninformationen.
Der globale Pfad hat dabei die bereits in der [Übersicht](#bersicht) gezeigte [Struktur](#message-struktur).

Die grundsätzliche Funktionsweise dieses Abschnitts lässt sich, zur Wahrung der Übersicht, in drei Teile
mit klaren Aufgabenbereichen untergliedern. Die des [Planners](#planner-path_provider), des [Replanners](#replanner-path_supervisor)
und des [Map Managers](#map-manager-common_road_manager). Eine genauere Erläuterung erfolgt in den jeweiligen Abschnitten.

#### Planner (path_provider)

<Beschreibung der Func>

#### Replanner (path_supervisor)

<Beschreibung der Func>

#### Map Manager (common_road_manager)

<Beschreibung der Func>

### Planning Preprocessor

<Beschreibung der Func>
