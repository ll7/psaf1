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
        this->state = LocalPlannerState::DRIVING;
    }

    void LocalPlannerStateMachine::reset() {
        this->state = LocalPlannerState::DRIVING;
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

    void LocalPlannerStateMachine::updateState(bool trafficLightDetected, bool stopDetected,
                                               psaf_messages::TrafficLight trafficLightKnowledge,
                                               double stoppingDistance,
                                               double currentSpeed, double distanceToStopLine, bool isIntersectionClear,
                                               double currentTimeSec) {
#ifdef STM_TRACE
        ROS_DEBUG("Update state based on:"
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
                }else if(stopDetected && trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_UNKNOWN){ // Seems to be in the wrong state -> go back to driving
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
                if (currentSpeed > 15 / 3.6 &&
                    // if traffic light knowledge is UNKNOWN we don't see any traffic light and are within the intersection or left it
                    (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_UNKNOWN
                    // If the traffic light is red go to DRIVING to reevaluate the decisions
                    ||trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED)) {
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    newState = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (distanceToStopLine < 2.0 || currentSpeed < 0.01) { // Keep 2m distance to stop line and accept speed of 0.01 as waiting
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
                if(trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_UNKNOWN){
                    // Update time since we haven't seen a traffic light while waiting
                    this->start_time_waiting_without_tl_state = std::min(this->start_time_waiting_without_tl_state,
                                                                         currentTimeSec);
                }else{
                    // Set value to default if we know the current traffic light state
                    this->start_time_waiting_without_tl_state = std::numeric_limits<double>::infinity();
                }
                // Check if we need an "emergency exit": We are waiting at the traffic light and have no knowledge about
                // traffic light state -> this is indicated by the fact that we haven't get any information about the
                // state for 15 seconds
                // The intersection must be clear to prevent a collision
                if(currentTimeSec-this->start_time_waiting_without_tl_state >= 15.0 && isIntersectionClear){
                    ROS_WARN("The state machine used the emergency exit while waiting at TL because TL state is unknown"
                             " for more than 15sec");
                    newState = LocalPlannerState::TRAFFIC_LIGHT_GO;
                }
                break;
                // Begin Stop sign / mark transitions
            case LocalPlannerState::STOP_NEAR:
                if (distanceToStopLine <= stoppingDistance) {
                    newState = LocalPlannerState::STOP_WILL_STOP;
                }else if(trafficLightDetected){ // Seems to be in the wrong state -> go back to driving
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::STOP_WILL_STOP:
                if (distanceToStopLine <= 2 || currentSpeed < 0.01) {
                    newState = LocalPlannerState::STOP_WAITING;
                    this->start_time_stop_waiting = currentTimeSec;
                }
                break;
            case LocalPlannerState::STOP_WAITING:
                // Wait until intersection is clear and wait at least 3 seconds
                if (isIntersectionClear && (currentTimeSec-this->start_time_stop_waiting)>1.) {
                    newState = LocalPlannerState::STOP_GO;
                    this->start_time_stop_go = currentTimeSec;
                }
                break;
            case LocalPlannerState::STOP_GO:
                if (currentSpeed > 15 / 3.6 && (currentTimeSec-this->start_time_stop_go)>=3.) {
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            default:
                // If Current state is not part of the transitions stay in current state
                break;
        }
        // Update state
        if(this->state!=newState){
            std::string oldChar = getTextRepresentation();
            this->state = newState;
            ROS_DEBUG("State changed from %s to %s",oldChar.c_str(),getTextRepresentation().c_str());
        }
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
                }else if(trafficLightDetected){ // Seems to be in the wrong state -> go back to driving
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::STOP_WILL_STOP:
                if (distanceToStopLine <= 2 || currentSpeed < 0.01) {
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
                if (currentSpeed > 15 / 3.6 && (currentTimeSec - this->start_time_stop_go) >= 3.) {
                    newState = LocalPlannerState::DRIVING;
                }
                break;
            default:
                // If Current state is not part of the transitions stay in current state
                break;
        }
        // Update state
        if(this->state!=newState){
            std::string oldChar = getTextRepresentation();
            this->state = newState;
            ROS_DEBUG("State changed from %s to %s",oldChar.c_str(),getTextRepresentation().c_str());
        }
    }
    bool LocalPlannerStateMachineWithoutTrafficRules::isInTrafficLightStates() {
        return LocalPlannerStateMachineWithoutTrafficRules::isInStopStates();
    }


    LocalPlannerStateMachineWithoutTrafficRules::~LocalPlannerStateMachineWithoutTrafficRules() = default;
}