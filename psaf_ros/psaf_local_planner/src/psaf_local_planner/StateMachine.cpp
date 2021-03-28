#include <psaf_local_planner/plugin_local_planner.h>
#include <psaf_local_planner/state_machine.h>


namespace psaf_local_planner {
    LocalPlannerStateMachine::LocalPlannerStateMachine() {
        this->state = LocalPlannerState::UNKNOWN;

        this->start_time_stop_waiting = 0.;
        this->start_time_waiting_without_tl_state = std::numeric_limits<double>::infinity();
        this->start_time_stop_go = std::numeric_limits<double>::infinity();
    }

    LocalPlannerStateMachine::~LocalPlannerStateMachine() = default;

    void LocalPlannerStateMachine::init() {
        reset();
    }

    void LocalPlannerStateMachine::reset() {
        setState(LocalPlannerState::DRIVING);
    }

    LocalPlannerState LocalPlannerStateMachine::getState() {
        return this->state;
    }


    bool LocalPlannerStateMachine::isInTrafficLightStates() {
        return this->state == LocalPlannerState::TRAFFIC_LIGHT_NEAR ||
               this->state == LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN ||
               this->state == LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP ||
               this->state == LocalPlannerState::TRAFFIC_LIGHT_WAITING ||
               this->state == LocalPlannerState::TRAFFIC_LIGHT_GO;
    }

    bool LocalPlannerStateMachine::isInStopStates() {
        return this->state == LocalPlannerState::STOP_NEAR ||
               this->state == LocalPlannerState::STOP_WILL_STOP ||
               this->state == LocalPlannerState::STOP_WAITING ||
               this->state == LocalPlannerState::STOP_GO;
    }

    void LocalPlannerStateMachine::setState(LocalPlannerState newState) {
        if (this->state != newState) {
            std::string oldChar = getTextRepresentation();
            this->state = newState;
            ROS_INFO_STREAM_NAMED(STM_LOGGER_NAME, "State changed from '" << oldChar << "' to '"
                                                                          << getTextRepresentation()<< "'.");
        }
    }

    void LocalPlannerStateMachine::updateState(bool trafficLightDetected, bool stopDetected,
                                               psaf_messages::TrafficLight trafficLightKnowledge,
                                               double stoppingDistance,
                                               double currentSpeed, double distanceToStopLine, bool isIntersectionClear,
                                               double currentTimeSec) {
#ifdef STM_TRACE
        ROS_DEBUG_NAMED(STM_LOGGER_NAME,"Update state based on:"
                  "TrafficLightDetected:%d"
                  "stopDetected:%d"
                  "trafficLightstate:%d"
                  "stoppingDistance:%f"
                  "currentSpeed:%f"
                  "distanceToStopLine:%f"
                  "isIntersectionClear:%d"
                  "currentTimeSec:%f",
                  trafficLightDetected,stopDetected,trafficLightKnowledge.state,stoppingDistance,
                  currentSpeed,distanceToStopLine,isIntersectionClear,currentTimeSec
                  );
#endif
        LocalPlannerState newState = this->state;
        // Switch statement represents transitions -> if no transition for current input is found we stay in the current state
        switch (this->state) {
            case LocalPlannerState::UNKNOWN:
                newState = LocalPlannerState::UNKNOWN;
                break;

            case LocalPlannerState::DRIVING:
                if (trafficLightDetected) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_NEAR;
                } else if (stopDetected) {
                    newState = LocalPlannerState::STOP_NEAR;
                }
                break;
                // Begin traffic light transitions
            case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN &&
                    distanceToStopLine < stoppingDistance) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED &&
                           distanceToStopLine < stoppingDistance) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED &&
                           distanceToStopLine >= stoppingDistance) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_YELLOW) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP;
                } else if (stopDetected && trafficLightKnowledge.state ==
                                           psaf_messages::TrafficLight::STATE_UNKNOWN) { // Seems to be in the wrong state -> go back to driving
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
                if (currentSpeed > SPEED_FOR_LEAVING_GO_STATE &&
                    // if traffic light knowledge is UNKNOWN we don't see any traffic light and are within the intersection or left it
                    (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_UNKNOWN
                     // If the traffic light is red go to DRIVING to reevaluate the decisions
                     || trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED)) {
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (distanceToStopLine < MIN_DISTANCE_TO_STOP_LINE || currentSpeed <
                                                                             EQUIVALENT_TO_0_VEL) { // Keep 2m distance to stop line and accept low speed as waiting
                    newState = LocalPlannerState::TRAFFIC_LIGHT_WAITING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED
                           && distanceToStopLine <= stoppingDistance) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_GO;
                }
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_UNKNOWN) {
                    // Update time since we haven't seen a traffic light while waiting
                    this->start_time_waiting_without_tl_state = std::min(this->start_time_waiting_without_tl_state,
                                                                         currentTimeSec);
                } else {
                    // Set value to default if we know the current traffic light state
                    this->start_time_waiting_without_tl_state = std::numeric_limits<double>::infinity();
                }
                // Check if we need an "emergency exit": We are waiting at the traffic light and have no knowledge about
                // traffic light state -> this is indicated by the fact that we haven't get any information about the
                // state for SEC_TO_ESCALATE_TO_EMERGENCY_EXIT seconds
                // The intersection must be clear to prevent a collision
                if (currentTimeSec - this->start_time_waiting_without_tl_state >= SEC_TO_ESCALATE_TO_EMERGENCY_EXIT
                    && isIntersectionClear) {
                    ROS_WARN_NAMED(STM_LOGGER_NAME,"The state machine used the emergency exit while waiting at TL because TL state is unknown"
                             " for more than %f sec",SEC_TO_ESCALATE_TO_EMERGENCY_EXIT);
                    newState = LocalPlannerState::TRAFFIC_LIGHT_GO;
                }
                break;
                // Begin Stop sign / mark transitions
            case LocalPlannerState::STOP_NEAR:
                if (distanceToStopLine <= stoppingDistance) {
                    newState = LocalPlannerState::STOP_WILL_STOP;
                } else if (trafficLightDetected) { // Seems to be in the wrong state -> go back to driving
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::STOP_WILL_STOP:
                if (distanceToStopLine <= MIN_DISTANCE_TO_STOP_LINE) {
                    newState = LocalPlannerState::STOP_WAITING;
                    this->start_time_stop_waiting = currentTimeSec;
                }
                break;
            case LocalPlannerState::STOP_WAITING:
                // Wait until intersection is clear and wait at least SEC_TO_WAIT_AT_STOP seconds
                if (isIntersectionClear && (currentTimeSec - this->start_time_stop_waiting) > SEC_TO_WAIT_AT_STOP
                    && currentSpeed <= EQUIVALENT_TO_0_VEL) {
                    newState = LocalPlannerState::STOP_GO;
                    this->start_time_stop_go = currentTimeSec;
                }
                break;
            case LocalPlannerState::STOP_GO:
                if (currentSpeed > SPEED_FOR_LEAVING_GO_STATE && (currentTimeSec - this->start_time_stop_go) >= SEC_TO_KEEP_STOP_GO) {
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            default:
                // If Current state is not part of the transitions stay in current state
                break;
        }
        // Update state
        this->setState(newState);
    }

    std::string LocalPlannerStateMachine::getTextRepresentation() {
        {
            switch (this->state) {
                case LocalPlannerState::DRIVING:
                    return "Driving";
                case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
                    return "Traffic light: near";
                case LocalPlannerState::TRAFFIC_LIGHT_GO:
                    return "Traffic light: go";
                case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
                    return "Traffic light: will stop";
                case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
                    return "Traffic light: slow down";
                case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
                    return "Traffic light: waiting";
                case LocalPlannerState::STOP_NEAR:
                    return "Stop: near";
                case LocalPlannerState::STOP_WILL_STOP:
                    return "Stop: will stop";
                case LocalPlannerState::STOP_WAITING:
                    return "Stop: waiting";
                case LocalPlannerState::STOP_GO:
                    return "Stop: go";
                case LocalPlannerState::UNKNOWN:
                    return "Unknown";
            }
            return "unknown state";
        }
    }

    // Begin of state machine definition without traffic rules

    LocalPlannerStateMachineWithoutTrafficRules::LocalPlannerStateMachineWithoutTrafficRules() = default;

    void LocalPlannerStateMachineWithoutTrafficRules::updateState(bool trafficLightDetected, bool stopDetected,
                                                                  psaf_messages::TrafficLight trafficLightKnowledge,
                                                                  double stoppingDistance,
                                                                  double currentSpeed, double distanceToStopLine,
                                                                  bool isIntersectionClear,
                                                                  double currentTimeSec) {
#ifdef LP_TRACE
        ROS_DEBUG("Update state based on:"
                  "TrafficLightDetected:%d"
                  "stopDetected:%d"
                  "trafficLightstate:%d"
                  "stoppingDistance:%f"
                  "currentSpeed:%f"
                  "distanceToStopLine:%f"
                  "isIntersectionClear:%d"
                  "currentTimeSec:%f",
                  trafficLightDetected, stopDetected, trafficLightKnowledge.state, stoppingDistance,
                  currentSpeed, distanceToStopLine, isIntersectionClear, currentTimeSec
        );
#endif
        LocalPlannerState newState = this->state;
        // Switch statement represents transitions -> if no transition for current input is found we stay in the current state
        switch (this->state) {
            case LocalPlannerState::UNKNOWN:
                newState = LocalPlannerState::UNKNOWN;
                break;

            case LocalPlannerState::DRIVING:
                if (trafficLightDetected) {
                    newState = LocalPlannerState::STOP_NEAR;
                } else if (stopDetected) {
                    newState = LocalPlannerState::STOP_NEAR;
                }
                break;
                // Begin traffic light transitions
            case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
            case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
            case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
                newState = LocalPlannerState::DRIVING;
                break;
                // Begin Stop sign / mark transitions
            case LocalPlannerState::STOP_NEAR:
                if (distanceToStopLine <= stoppingDistance) {
                    newState = LocalPlannerState::STOP_WILL_STOP;
                }
                break;
            case LocalPlannerState::STOP_WILL_STOP:
                if (distanceToStopLine <= MIN_DISTANCE_TO_STOP_LINE) {
                    newState = LocalPlannerState::STOP_WAITING;
                }
                break;
            case LocalPlannerState::STOP_WAITING:
                if (isIntersectionClear) {
                    newState = LocalPlannerState::STOP_GO;
                    this->start_time_stop_go = currentTimeSec;
                }
                break;
            case LocalPlannerState::STOP_GO:
                if (currentSpeed > SPEED_FOR_LEAVING_GO_STATE && (currentTimeSec - this->start_time_stop_go) >= SEC_TO_KEEP_STOP_GO) {
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            default:
                // If Current state is not part of the transitions stay in current state
                break;
        }
        // Update state
        this->setState(newState);
    }

    bool LocalPlannerStateMachineWithoutTrafficRules::isInTrafficLightStates() {
        return false;
    }


    LocalPlannerStateMachineWithoutTrafficRules::~LocalPlannerStateMachineWithoutTrafficRules() = default;
}