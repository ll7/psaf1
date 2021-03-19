# Costmap Raytracer
###### [Source (.cpp)](src/psaf_local_planner/costmap_raytracer.cpp)  | [Header (.h)](include/psaf_local_planner/costmap_raytracer.h)
Der Costmap Raytracer findet durch Raytracing Hindernisse auf der Costmap innerhalb eines Teilkreises mit definiertem Radius und Winkeln um das Auto herum (vgl. Abb unten).
Durch eine niedrige Schrittweite zwischen den Rays, können einzelne Fahrzeuge teilweise mehrmals getroffen werdnen. So wird die Position des Hindernisses, der Abstand zum Fahrzeug sowie die Seite des Fahrzeugs, auf der sich das Hinderniss befindet, ermittelt.


```
         │       o┌─┐
   ╲     │      ╱ │A│
    ╲    │     ╱  └─┘o
_    ╲   │    ╱   _╱
 ╲_   ╲  │   ╱  _╱
   ╲_  ╲ │  ╱ _╱
     ╲_ ╭─╮ _╱       ┌─┐
─────── │C│ ────────o│A│
(-)min  ╰─╯  max(+)  └─┘
```

Zur Überprüfung ob z.B. Stopkreuzungen [TODO: Link zu Stopkreuzungen in Doku]() frei sind ist eine zeitliche Abhängigkeit nötig. Durch Abgleichen mit den Messungen zwei vorheriger Iterationen, kann festgestellt werden, ob Bewegungen auf der Costmap auftreten. Werden keine Veränderungen erkannt, wird der Bereich als passierbar freigegeben, um statische Hindernisse ignorieren zu können.

# Local Perception
###### [Source (.cpp)](src/psaf_local_planner/LocalPerception.cpp)  | [Header (.h)](include/psaf_local_planner/plugin_local_planner.h#L223)
Die Local Perception vehindert anhand der Costmap die Kollision mit vorausfahrenden bzw. haltenden Fahrzeugen. Hierfür wird die Distanz vom eingenen Fahrzeug bis zum nächsten Hinderniss gemessen.
So kann mithilfe der Formel für den [Anhalteweg](https://www.bussgeldkatalog.org/anhalteweg/), die nach der Geschwindigkeit umgestellt wird, die nötige Geschwindigkeit herausgefunden werden, die das Fahrzeug ansteuern muss, um einen konstanten Abstand zu halten.

Unterschreitet diese Geschwindigkeit über einen längeren Zeitraum einen bestimmten Schwellenwert, dann wird eine Spurwechsel initiert. Hierfür werden alle durch den [Costmap Raytracer](#Costmap-Raytracer) erkannten Hindernisse in der [Obstacle](../psaf_messages/msg/Obstacle.msg) Message an den Global Planner gesendet.
