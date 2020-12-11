# CommonRoad Route Planner

This repository hosts the code base of a commonly used route planner. The basic functionality is to find a sequence of lanelets (route) that leads from the initial lanelet(s)  to the goal lanelet(s) of the planning problem. It also works on survival scenarios (where no goal is specified). Additionally, it also constructs a reference path for each planned route.

### Backend
The planner supports different backends to search for the shortest route in the scenario:
1. NETWORKX: uses built-in functions from the networkx package, tends to change lane later
2. NETWORKX_REVERSED: uses built-in functions from the networkx package, tends to change lane earlier
3. PRIORITY_QUEUE: uses A-star search to find routes, lane change maneuver depends on the heuristic cost
## Tutorial
A tutorial can be found at `tutorial/tutorial_route_planner.ipynb`.
