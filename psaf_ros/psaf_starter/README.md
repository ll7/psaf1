# psaf_starter


Dieses Paket ermöglicht das komfortable Starten unseres Projektes. 
Dafür stehen zwei launch-Dateien zur Verfügung:
* **start_with_traffic_rules.launch**
* **start_without_traffic_rules.launch**

Beide Dateien starten dabei alle benötigten Komponenten des Projekts. Es wird lediglich unterschieden,
ob das Fahrzeug Verkehrsregeln, wie Ampeln oder Geschwindigkeitsbeschränkungen beachten 
soll. Diese Unterscheidung geschieht mit dem in den launch-Dateien gesetztem Parameter 
**obeyTrafficRules**. Für die Fahrt ohne Verkehrsregeln erwägt das Fahrzeug nach dem 
Start einen U-Turn, wenn dadurch das Ziel schneller erreicht werden kann. 
Dieses Verhalten kann mit dem Parameter **alwaysUTurn** auch bei der Fahrt mit 
Verkehrsregeln erzwungen werden. Beide Parameter werden jeweils an den entsprechenden 
Stellen von anderen Paketen ausgelesen.
