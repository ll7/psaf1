from enum import Enum, auto
from transitions.extensions import HierarchicalGraphMachine as Machine
from transitions.extensions.nesting import NestedState

NestedState.separator = '↦'

"""
The top level states
"""
DRIVING_ON_ROAD = "DRIVING_ON_ROAD"
TRAFFIC_LIGHT_NEAR = "TRAFFIC_LIGHT_NEAR "
TRAFFIC_LIGHT_GO = "TRAFFIC_LIGHT_GO"
TRAFFIC_LIGHT_WILL_STOP = "TRAFFIC_LIGHT_WILL_STOP"
TRAFFIC_LIGHT_SLOW_DOWN = "TRAFFIC_LIGHT_SLOW_DOWN"
TRAFFIC_LIGHT_WAITING = "TRAFFIC_LIGHT_WAITING"

"""
Transitions
"""
T_TRAFFIC_LIGHT_YELLOW = "trafficLightYellow"
T_TRAFFIC_LIGHT_RED = "trafficLightRed"
T_TRAFFIC_LIGHT_GREEN = "trafficLightGreen"
T_TRAFFIC_LIGHT_DETECTED = "trafficLightDetected"
T_CLOSE_TO_STOP_LINE = "arrivedAtStopLine2M"

""" 
Conditions
"""
IS_DISTANCE_BELOW_AW = "is_traffic_light_distance_below_stopping_distance"


def get_state_machine(model):
    """
    Creates a state machine to handle the traffic rules inside the local traffic planner
    :param model: the model for the states
    :return: the state machine
    """
    states = []
    states.append(NestedState(DRIVING_ON_ROAD))
    states.append(NestedState(TRAFFIC_LIGHT_NEAR))
    states.append(NestedState(TRAFFIC_LIGHT_GO))
    states.append(NestedState(TRAFFIC_LIGHT_WILL_STOP))
    states.append(NestedState(TRAFFIC_LIGHT_SLOW_DOWN))
    states.append(NestedState(TRAFFIC_LIGHT_WAITING))

    machine = Machine(model=model,states=states,initial=DRIVING_ON_ROAD)

    machine.add_transition(T_TRAFFIC_LIGHT_DETECTED, DRIVING_ON_ROAD, TRAFFIC_LIGHT_NEAR)
    # Leaving traffic light near
    machine.add_transition(T_TRAFFIC_LIGHT_GREEN, TRAFFIC_LIGHT_NEAR, TRAFFIC_LIGHT_GO,
                           conditions=[IS_DISTANCE_BELOW_AW])
    machine.add_transition(T_TRAFFIC_LIGHT_GREEN, TRAFFIC_LIGHT_NEAR, None, unless=[IS_DISTANCE_BELOW_AW])
    machine.add_transition(T_TRAFFIC_LIGHT_RED, TRAFFIC_LIGHT_NEAR, TRAFFIC_LIGHT_WILL_STOP,
                           conditions=[IS_DISTANCE_BELOW_AW])
    machine.add_transition(T_TRAFFIC_LIGHT_YELLOW, TRAFFIC_LIGHT_NEAR, TRAFFIC_LIGHT_WILL_STOP)
    machine.add_transition(T_TRAFFIC_LIGHT_RED, TRAFFIC_LIGHT_NEAR, TRAFFIC_LIGHT_SLOW_DOWN,
                           unless=[IS_DISTANCE_BELOW_AW])
    # leaving GO
    machine.add_transition("speed_above_10kmh", TRAFFIC_LIGHT_GO, DRIVING_ON_ROAD)
    # leaving slow down
    machine.add_transition(T_TRAFFIC_LIGHT_RED, TRAFFIC_LIGHT_SLOW_DOWN, TRAFFIC_LIGHT_WILL_STOP)
    machine.add_transition(T_TRAFFIC_LIGHT_GREEN, TRAFFIC_LIGHT_SLOW_DOWN, TRAFFIC_LIGHT_GO)
    # leaving will stop
    machine.add_transition(T_TRAFFIC_LIGHT_GREEN, TRAFFIC_LIGHT_WILL_STOP, TRAFFIC_LIGHT_GO)
    machine.add_transition(T_CLOSE_TO_STOP_LINE, TRAFFIC_LIGHT_WILL_STOP, TRAFFIC_LIGHT_WAITING)
    # leaving waiting
    machine.add_transition(T_TRAFFIC_LIGHT_GREEN, TRAFFIC_LIGHT_WAITING, TRAFFIC_LIGHT_GO)

    return machine


# Test the state machine creation and draw it

if __name__ == '__main__':
    class DummyModel:
        def __init__(self):
            self.name = "dummy"

        def is_traffic_light_distance_below_stopping_distance(self):
            return True

    m = DummyModel()
    machine = get_state_machine(m)
    m.trafficLightDetected()
    m.trafficLightRed()
    m.get_graph().draw('my_state_diagram.png', prog='dot')
