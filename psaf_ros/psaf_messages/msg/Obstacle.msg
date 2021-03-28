#######################################################################################################################
# Message that contains the information about a obstacle located on the road
#######################################################################################################################

# id planning msg
uint8 id

#The list of obstacles relative to the car position
geometry_msgs/Point[] obstacles
