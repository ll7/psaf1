# psaf_optimizer

## Inhalt

* [Inhalt](#inhalt)
* [Übersicht](#bersicht)
    * [Kurzbeschreibung](#kurzbeschreibung)
    * [Launch Files](#launch-files)
* [Funktionalität](#funktionalitt)
    * [Optimizer](#optimizer-simulated_annealing_optimizer)

## Übersicht

### Kurzbeschreibung
Die Hauptaufgabe des Optimizer Packages, ist die Ermöglichung einer Optimierung verschiedener Parameter anhand eines vom
[Scenario Runner](../psaf_scenario/README.md) bereitgestellten Szenarios.

### Launch Files
- **psaf_optimizer_run_optimizer.launch** startet ```simulated_annealing_optimizer.py```; Diese stellt eine Python
  Implementierung eines Simulated Annealing Optimizers dar. Sämtliche Hyperparameter des Optimierers können über die 
  Launch Datei gesetzt werden.

## Funktionalität
### Optimizer (simulated_annealing_optimizer)
Der Simulated Annealing Optimizer basiert im Grunde auf einem Random Search Optimizer, dessen Explorations-, beziehungsweise
Exploitations-Verhalten von einer Abkühlungsfunktion beeinflusst wird. Diese simuliert den Prozess des Abkühlens von 
Metallen. In Folge wird mit fortschreitender Zeit mehr die Exploitation anstelle der Exploration forciert.
Das Parameterset mit dem besten erreichten Score wird nach jeder Iteration gesichert.
In jeder Iteration werden die vom Optimierer gewählten Parameter zur Laufzeit gesetzt und vom Scenario Runner
evaluiert. Der o.g. Score setzt sich aus einer anpassbaren Gewichtung aus der Routenqualität und der benötigten Zeit zusammen.
Für eine detailliertere Beschreibung dieser Metriken ist auf den Ort der Generierung, 
dem [Scenario Runner](../psaf_scenario/README.md), zu verweisen.

In der aktuellen Fassung werden nur die PID Parameter der Geschwindigkeits- und Beschleunigungs-Regelung optimiert.
Prinzipiell ist eine Erweiterung zur Optimierung anderer Parameter leicht möglich. Dazu müsste lediglich der Parameter-Typ
zur ```ParameterType``` Enumeration hinzugefügt werden, sowie die ```_set_params``` angepasst werden.
Eine Erweiterung der ```_check_parameter_validity``` erlaubt das Überprüfen auf korrekte Übergabe-Parameter, da neben 
dem zu optimierenden Parametertyp, auch die Anzahl der Parameter, sowie deren Wertebereich übergeben werden muss.
Anhand von diesen Informationen wird dann in jeder Iteration, in Abhängigkeit der absoluten Größe des Wertebereichs, für 
jeden Parameter die Schrittweite bestimmt.