Aus map kann waypointe rausgelesen werden:
https://carla.readthedocs.io/en/latest/python_api/#carla.Waypoint
--> `get_landmarks_of_type(self, distance, type, stop_at_junction=False)`

Bei Landmarks über

```
type (str)
Type identificator of the landmark according to the country code.
sub_type (str)
Subtype identificator of the landmark according to the country code.
value (float)
Value printed in the signal (e.g. speed limit, maximum weight, etc). 
```

Kann theoretisch max geschwindigkeit rauslesen
