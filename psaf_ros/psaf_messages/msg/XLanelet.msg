#######################################################################################################################
# Message that contains the information that is needed start/update a p2p navigation
#######################################################################################################################

# infos
bool hasLight
bool isAtIntersection
bool hasStop
uint8 id

# The list of obstacles relative to the car position
CenterLineExtended[] route_portion
