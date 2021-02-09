#######################################################################################################################
# Message that contains the information that is needed start/update a p2p navigation
#######################################################################################################################

# id planning msg
uint32 id

# is u_turn
bool isUTurn

# List of all lanlets, that are located on the route
XLanelet[] route
