#include "psaf_local_planner/plugin_local_planner.h"


namespace psaf_local_planner {

    void PsafLocalPlanner::updateStateMachine() {
        bool traffic_light_detected = false;

        if (global_route.size() > 0) {
            if (global_route[0].route_portion.size() > 0) {
                traffic_light_detected = global_route[0].hasLight;
            }
            // TODO check next lanelet if current is too short
        }

        this->state_machine->updateState(traffic_light_detected, this->traffic_light_state,
                                         this->getCurrentStoppingDistance(), this->current_speed, this->stop_distance_at_intersection);
    }

    double PsafLocalPlanner::getTargetVelIntersection() {
        double target_vel = target_velocity;
        switch (this->state_machine->getState()) {
            case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
                target_vel = getMaxVelocity();
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
                double velocity_distance_diff;
                if (stop_distance_at_intersection < 3) {
                    velocity_distance_diff = target_velocity;
                } else {
                    velocity_distance_diff = target_velocity - std::min(target_velocity, 25.0 / 18.0 * (-1 + std::sqrt(
                            1 + 4 * (stop_distance_at_intersection - 2))));
                }
                target_vel = target_velocity - velocity_distance_diff;
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
                target_vel = target_velocity / 2;
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
                target_vel = 0.0;
                break;
            default:
                target_vel = target_velocity;
        }
        return std::min(target_vel, target_velocity);
    }

}