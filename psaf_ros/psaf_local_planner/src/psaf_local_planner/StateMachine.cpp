#include <psaf_local_planner/plugin_local_planner.h>


namespace psaf_local_planner {
    LocalPlannerStateMachine::LocalPlannerStateMachine() {
        this->state = LocalPlannerState::UNKNOWN;
    }

    void LocalPlannerStateMachine::init() {
        this->state = LocalPlannerState::DRIVING;
    }

    LocalPlannerState LocalPlannerStateMachine::getState() {
        return this->state;
    }

    void LocalPlannerStateMachine::setState(LocalPlannerState state) {
        this->state = state;
    }

    void LocalPlannerStateMachine::updateState(bool trafficLightDetected, 
                                               psaf_messages::TrafficLight trafficLightKnowledge,
                                          double stoppingDistance, double currentSpeed, double distanceToStopLine) {

        switch (this->state) {
            case LocalPlannerState::UNKNOWN:
                this->state = LocalPlannerState::UNKNOWN;
                break;

            case LocalPlannerState::DRIVING:
                ROS_WARN("STATE IS DRIVING");
                if (trafficLightDetected) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_NEAR;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
                ROS_WARN("STATE IS NEAR Light");
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN &&
                    trafficLightKnowledge.distance < stoppingDistance) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED &&
                           trafficLightKnowledge.distance < stoppingDistance) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP;
                }
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED &&
                    trafficLightKnowledge.distance >= stoppingDistance) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_YELLOW) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
                ROS_WARN("STATE IS GO");
                if (currentSpeed > 2.8) {
                    this->state = LocalPlannerState::DRIVING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
                ROS_WARN("STATE IS WILL STOP");
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (distanceToStopLine < 2.0) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_WAITING;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
                ROS_WARN("STATE IS SLOW DOWN");
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_GO;
                } else if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_RED) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP;
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
                ROS_WARN("STATE IS WAITING");
                if (trafficLightKnowledge.state == psaf_messages::TrafficLight::STATE_GREEN) {
                    this->state = LocalPlannerState::TRAFFIC_LIGHT_GO;
                }
                break;
            default:
                break;
        }
    }

}