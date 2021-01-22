#######################################################################################################################
# Message that contains the information about a detected traffic light
#######################################################################################################################


#The relative position seen from the upper left corner
float32 x
float32 y

# distance in meters
float32 distance

# the state of the traffic light. see constants for interpretation
uint8 state

# constants representing the traffic light state
uint8 STATE_UNKNOWN=0
uint8 STATE_RED=1
uint8 STATE_YELLOW=2
uint8 STATE_GREEN=3