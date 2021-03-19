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
Die Hauptaufgabe des Optimizer Packages, ist die Ermöglichung einer Optimierung verschiedener Parameter anhand eines von
[psaf_scenario](../psaf_scenario/readme.md) bereitgestellten Scenarios.

### Launch Files
- **psaf_optimizer_run_optimizer.launch** startet den ```simulated_annealing_optimizer.py```; Dieser stellt eine Python
  Implementierung eines Simulated Annealing Optimizers dar. Sämtliche Hyperparameter des Optimierers können über diese 
  Launch Datei gesetzt werden.

## Funktionalität
### Optimizer (simulated_annealing_optimizer)
Der Simulated Annealing Optimizer basiert im Grunde auf einem Random Search Optimizer, dessen Explorations-, beziehungsweise
Exploitations-Verhalten von einer Abkühlungsfunktion beeinflusst wird. Diese simuliert den Prozess des Abkühlens von 
Metallen. In Folge wird mit fortschreitender Zeit mehr das Exploitation anstelle der Exploration forciert. 