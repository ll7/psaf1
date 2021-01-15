#######################################################################################################################
# Message that contains the information about a obstacle located on the road
#######################################################################################################################

# id of the obstacle used for removal action
uint8 id

# action for the obstacle see constants
uint8 action

#The position relative to the car position
float32 x
float32 y

# current speed of the obstacle in m/s
float32 speed

# constants representing the action that should be taken for the obstacle
uint8 ACTION_ADD = 0
uint8 ACTION_REMOVE = 1