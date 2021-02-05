#######################################################################################################################
# Message that passes the goal information from local to the global planner
#######################################################################################################################


#The goal point in GPS coords
sensor_msgs/NavSatFix goalPoint
# Whether a u-turn is possible at the current start pose
bool uTurnPossible