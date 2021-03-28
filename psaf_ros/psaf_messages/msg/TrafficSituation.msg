#######################################################################################################################
# Message that informs the subscriber about the current traffic situation
#######################################################################################################################

# The header: Time, sequence number etc
Header header


# Detected traffic light state. empty if nothing has been seen. max size 1
TrafficLight[] trafficLight

# Distance to stop line in meters, infinity if no stop line in front of car
float32 distanceToStopLine

