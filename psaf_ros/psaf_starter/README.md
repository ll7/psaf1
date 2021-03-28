# psaf_starter


Dieses Paket ermöglicht das komfortable Starten unseres Projektes. 
Dafür stehen zwei launch-Dateien zur Verfügung:
* [**start_with_traffic_rules.launch**](launch/start_with_traffic_rules.launch)
* [**start_without_traffic_rules.launch**](launch/start_without_traffic_rules.launch)

Beide Dateien verweisen dabei auf die intern benötigte launch-Datei [**__start_autonomous_driving_base.launch**](launch/__start_autonomous_driving_base.launch),
die alle benötigten Komponenten des Projekts startet. Es wird lediglich unterschieden,
ob das Fahrzeug Verkehrsregeln, wie Ampeln oder Geschwindigkeitsbeschränkungen beachten 
soll. Diese Unterscheidung geschieht mit dem in den beiden launch-Dateien gesetztem Parameter 
**obeyTrafficRules**. Für die Fahrt ohne Verkehrsregeln erwägt das Fahrzeug nach dem 
Start einen U-Turn, wenn dadurch das Ziel schneller erreicht werden kann. 
Dieses Verhalten kann mit dem Parameter **alwaysUTurn** in [**__start_autonomous_driving_base.launch**](launch/__start_autonomous_driving_base.launch) auch bei der Fahrt mit 
Verkehrsregeln erzwungen werden. 
 
Parameter, die die Fahrt mit und ohne Verkehrsregeln betreffen, können in der Datei [**__start_autonomous_driving_base.launch**](launch/__start_autonomous_driving_base.launch)
konfiguriert werden:

| **Parameter**  | **Standardwert**     | **Beschreibung**                                             |
|----------------|----------------------|--------------------------------------------------------------|
| use_gpu        | True                 | Verwendung der GPU für [PSAF Perception](../psaf_perception) |
| always_u_turn  | True                 | Versucht immer U-Turn zu planen unabhängig von **obeyRules** |
| role_name      | ego_vehicle          | Name des Fahrzeuges                                          |
| vehicle_filter | vehicle.tesla.model3 | Fahrzeugmodell                                               |
| town           | Town03               | CARLA Karte                                                  |


