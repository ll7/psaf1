# psaf_local_planner

## Inhalt
* [Inhalt](#inhalt)
* [Übersicht](#bersicht)
    * [Kurzbeschreibung](#kurzbeschreibung)
    * [Topics](#topics)
    * [Message Struktur](#message-struktur)
* [Funktionalität](#funktionalitt)
    * [Global Plan Verarbeitung](#global-plan-verarbeitung)
    * [Driving](#driving)

## Übersicht
### Kurzbeschreibung
Die Hauptaufgabe des Local Planner ist das Berechnen einer Geschwindigkeits- und einer Lenkwinkelvorgabe für das Fahrezug. 
Die Entscheidung zur Festlegung dieser Vorgaben wird dabei Anhand der zur Verfügung stehenden Informationen wie beispielsweise der Globalen Route (Geschwindigkeit auf Streckenabschnitten) oder der Perception (Ampeln, Stoppschilder) geschehen. 
Dabei werden andere Fahrzeuge im näheren Umfeld des eigenen Fahrzeugs auch in diese Planung einbezogen. 
Des Weiteren entscheidet der Local Planer auch auf Basis der Informationen ob eine Neuplanung des GlobalPlanners ausgelöst werden soll.
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
Die vom GlobalPlanner erstellte Global Route wird zur weiteren Verarbeitung im LocalPlaner lokal gespeichert.
Dabei wird gewährleistet, dass die lokale Route aktualisiert bleibt, indem bereits überfahrende Teile der lokalen Route gelöscht werden.
Dieser Lokale Plan wird an Spurwechseln durch lineare Interpolation der einzelnen Pfad Punkte so angepasst, dass sich abhängig von der erlaubten Geschwindigkeit ein weicher Spurwechsel ergibt.
Handelt es sich beim gepublishten Globalen Plan um ein Replaning wird mithilfe einer Herustik entschieden ob die neue Route doppelt so lang wie die aktuelle lokale XRoute ist.
Dies wird abhängig von der Beachtung der Verkerhsregeln anhand der Duration in der XRoute(Beachtung der Verkehrsregeln) oder der Länge der Lanelets (keine Beachtung der Verkerhsregeln) entschieden.
Falls dieser neue Globale PLan zu lang ist, wird weiterhin der alte lokale Plan beibehalten.
### Driving


### LocalPerception

