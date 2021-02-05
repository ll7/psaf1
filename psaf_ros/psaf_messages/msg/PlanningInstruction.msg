#######################################################################################################################
# Message that passes the goal information from local to the global planner
#######################################################################################################################


#The goal point in GPS coords
sensor_msgs/NavSatFix goalPoint
# global planner should plan U-Turn
bool planUTurn
# distance to next obstacle on the left/forward to check if U-Turn is possible
float32 obstacleDistanceForward
float32 obstacleDistanceLeft