#######################################################################################################################
# Message that informs the subscriber about the traffic signs that have beed detected by the perception module
#######################################################################################################################

# The header: Time, sequence number etc
Header header

# All detected speed signs
SpeedSign[] speedSigns

# All detected stop marks
StopMark[] stopMarks

# All detected stop signs
StopSign[] stopSigns

# All detected traffic lights
TrafficLight[] trafficLights