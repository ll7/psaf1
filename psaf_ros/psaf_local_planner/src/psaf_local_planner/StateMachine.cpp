#include <psaf_local_planner/plugin_local_planner.h>
#include <psaf_local_planner/state_machine.h>


namespace psaf_local_planner {
    LocalPlannerStateMachine::LocalPlannerStateMachine() {
        this->state = LocalPlannerState::UNKNOWN;
    }

    LocalPlannerStateMachine::~LocalPlannerStateMachine() {

    }

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
                                               double currentSpeed, double distanceToStopLine,
                                               bool isIntersectionClear) {

        // Switch statement represents transitions -> if no transition for current input is found we stay in the current state
        switch (this->state) {
            case LocalPlannerState::UNKNOWN:
                this->state = LocalPlannerState::UNKNOWN;
                break;

            case LocalPlannerState::DRIVING:
                if (trafficLightDetected) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_NEAR;
                } else if (stopDetected) {
                    this->state = LocalPlannerState::STOP_NEAR;
                }
                break;
                // Begin traffic light transitions
            case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN &&
                    distanceToStopLine < stoppingDistance) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED &&
                           distanceToStopLine < stoppingDistance) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED &&
                    distanceToStopLine >= stoppingDistance) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_YELLOW) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP;
                }else if(stopDetected){ // Seems to be in the wrong state -> go back to driving
                    this->state = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
                if (currentSpeed > 15 / 3.6 &&
                    // if traffic light knowledge is UNKNOWN we don't see any traffic light and are within the intersection or left it
                    (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_UNKNOWN
                    // If the traffic light is red go to DRIVING to reevaluate the decisions
                    ||trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED)) {
                    this->state = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (distanceToStopLine < 2.0 || currentSpeed < 0.01) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_WAITING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED
                           && distanceToStopLine <= stoppingDistance) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_GO;
                }
                break;
                // Begin Stop sign / mark transitions
            case LocalPlannerState::STOP_NEAR:
                if (distanceToStopLine <= stoppingDistance) {
                    this->state = LocalPlannerState::STOP_WILL_STOP;
                }else if(trafficLightDetected){ // Seems to be in the wrong state -> go back to driving
                    this->state = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::STOP_WILL_STOP:
                if (distanceToStopLine <= 2 || currentSpeed < 0.01) {
                    this->state = LocalPlannerState::STOP_WAITING;
                }
                break;
            case LocalPlannerState::STOP_WAITING:
                if (isIntersectionClear) {
                    this->state = LocalPlannerState::STOP_GO;
                }
                break;
            case LocalPlannerState::STOP_GO:
                if (currentSpeed > 15 / 3.6) {
                    this->state = LocalPlannerState::DRIVING;
                }
                break;
            default:
                // If Current state is not part of the transitions stay in current state
                break;
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
            }
            return "unknown state";
        }
    }

}