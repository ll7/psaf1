# Costmap Raytracer
###### [Source (.cpp)](src/psaf_local_planner/costmap_raytracer.cpp)  | [Header (.h)](include/psaf_local_planner/costmap_raytracer.h)

Der Costmap Raytracer findet durch Raytracing Hindernisse auf der Costmap innerhalb eines Teilkreises mit definiertem Radius und Winkeln um das Auto herum (vgl. Abb unten).
Durch eine niedrige Schrittweite zwischen den Rays, können einzelne Fahrzeuge teilweise mehrmals getroffen werdnen. So wird die Position des Hindernisses, der Abstand zum Fahrzeug ermittelt sowie die Seite des Fahrzeugs auf der sich das Hinderniss befindet.


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