# psaf_local_planner

## Inhalt
* [Inhalt](#inhalt)
* [Übersicht](#%c3%9cbersicht)
    * [Kurzbeschreibung](#kurzbeschreibung)
    * [Topics](#topics)
    * [Message Struktur](#message-struktur)
* [Funktionalität](#func)
    * [Global Plan Verarbeitung](#global-plan-verarbeitung)
    * [Driving](#driving)

## Übersicht
### Kurzbeschreibung

### Topics
#### Publish
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| psaf_global_plan | Path | Global Plan Verarbeitung |
| /psaf/planning/obstacle | Obstacle |  Local Perception |
| /psaf/debug/local_planner/state | String |  Local Perception |


#### Subscribe
| Topic | Datatype | Module|
| ----------- | ----------- |----------- |
| /psaf/xroute | XRoute | Global Plan Verarbeitung |
| /psaf/local_planner/traffic_situation | TrafficSituation |  Local Perception Evaluation |
| /carla/ego_vehicle/vehicle_status | CarlaEgoVehicleStatus | State Machine | 
### Message Struktur


## Funktionalität
### Global Plan Verarbeitung
### Driving
### LocalPerception

