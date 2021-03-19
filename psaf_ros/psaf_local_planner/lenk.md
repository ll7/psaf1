# Lenkwinkelvorgabe
###### [Source (.cpp)](src/psaf_local_planner/Driving.cpp)  | [Header (.h)](include/psaf_local_planner/plugin_local_planner.h#L175)

Der Lenkwinkel des Fahrzeugs wird anhand der aktuellen Position des Fahrezeugs sowie einem Punkt auf dem lokalen Pfad. Dieser Punkt wird abhängig von der Geschwindigkeit einige Meter vor dem Fahrzeug gewählt. Ein höherer Abstand sorgt für ein glatteres Fahrverhalten mit weniger Oszillationen, führt jedoch dazu, dass Kurven zu eng genommen bzw. geschnitten werden.

Mit Hilfe des `atan2` wird der Winkel zwischen dem Fahrzeugsvektor sowie dem Vektor vom Fahrzeugsmittelpunkt zum Zielpunkt im Interval [-pi, +pi] berechnet. Dieser Winkel entspricht dem Lenkwinkel des Fahrzeugs.


```
               x  x
         x  x 
       o 
 |   x╱
 │ x ╱ 
 │x ╱
╭─╮╱
│C│
╰─╯
```



# Kurvenmaximalgeschwindigkeit
###### [Source (.cpp)](src/psaf_local_planner/Driving.cpp)  | [Header (.h)](include/psaf_local_planner/plugin_local_planner.h#L190)

Um nicht aus engen Kurven zu fliegen, muss die dort maximal mögliche Geschwindigkeit im Vorraus berechnet werden. Hierfür kann die Haftreibungsformel in Kurven verwendet werden: `v <= sqrt(µ_haft * r * g)`.

Hierfür ist jedoch der Kreisradius der Kurve nötig. Dieser kann mit der [Menger Gleichung](https://en.wikipedia.org/wiki/Menger_curvature#Definition) berechnet werden. Hierfür werden drei Punkte auf der nächsten Kurve gesucht. Der Anfang bzw. das Ende der Kurve wird gefunden durch ein Vorzeichenwechsel zwischen dem Winkel drei aufeinanderfolgenden Punkten.

```
               o x x x x x
          x x
      x o 
    x
  x
 x
o
x
x
x
```