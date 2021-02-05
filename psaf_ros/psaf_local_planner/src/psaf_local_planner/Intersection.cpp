#include "psaf_local_planner/plugin_local_planner.h"


namespace psaf_local_planner {

    double PsafLocalPlanner::computeDistanceToUpcomingTrafficLight() {

        if (global_route.size() > 0) {
            if (global_route[0].route_portion.size() > 1) {
                double remaining_way_on_lanelet =
                        this->distanceBetweenCenterLines(global_route[0].route_portion.front(),
                                                         global_route[0].route_portion.back());
                // if the lanelet is too short we look on the next lanelet
                if (remaining_way_on_lanelet > getCurrentStoppingDistance()
                    && (remaining_way_on_lanelet < 1.5 * getCurrentStoppingDistance())) {
                    // If the remaining distance is smaller than the 1.5*stopping distance we check if there is a traffic light
                    if(global_route[0].hasLight){
                        return remaining_way_on_lanelet;
                    }
                    return std::numeric_limits<double>::infinity();
                }
                // Check next lanelet if there is one
                if (global_route.size() > 1 && global_route[1].route_portion.size() > 1) {
                    double remaining_way_to_next_lanelet_end = remaining_way_on_lanelet +
                            this->distanceBetweenCenterLines(global_route[1].route_portion.front(),
                                                             global_route[1].route_portion.back());
                    if (remaining_way_to_next_lanelet_end > getCurrentStoppingDistance()
                        && (remaining_way_on_lanelet < 1.5 * getCurrentStoppingDistance())) {
                        // If the remaining distance is smaller than the 1.5*stopping distance we check if there is a traffic light
                        if(global_route[0].hasLight){
                            return remaining_way_to_next_lanelet_end;
                        }
                        return std::numeric_limits<double>::infinity();
                    }
                }
            }
        }
        return false;
    }

    double PsafLocalPlanner::computeDistanceToUpcomingStop() {

        if (global_route.size() > 0) {
            if (global_route[0].route_portion.size() > 1) {
                double remaining_way_on_lanelet =
                        this->distanceBetweenCenterLines(global_route[0].route_portion.front(),
                                                         global_route[0].route_portion.back());
                // if the lanelet is too short we look on the next lanelet
                if (remaining_way_on_lanelet > getCurrentStoppingDistance()
                    && (remaining_way_on_lanelet < 1.5 * getCurrentStoppingDistance())) {
                    // If the remaining distance is smaller than the 1.5*stopping distance we check if there is a traffic light
                    if(global_route[0].hasStop){
                        return remaining_way_on_lanelet;
                    }
                    return std::numeric_limits<double>::infinity();
                }
                // Check next lanelet if there is one
                if (global_route.size() > 1 && global_route[1].route_portion.size() > 1) {
                    double remaining_way_to_next_lanelet_end = remaining_way_on_lanelet +
                                                               this->distanceBetweenCenterLines(global_route[1].route_portion.front(),
                                                                                                global_route[1].route_portion.back());
                    if (remaining_way_to_next_lanelet_end > getCurrentStoppingDistance()
                        && (remaining_way_on_lanelet < 1.5 * getCurrentStoppingDistance())) {
                        // If the remaining distance is smaller than the 1.5*stopping distance we check if there is a traffic light
                        if(global_route[0].hasStop){
                            return remaining_way_to_next_lanelet_end;
                        }
                        return std::numeric_limits<double>::infinity();
                    }
                }
            }
        }
        return false;
    }

    void PsafLocalPlanner::updateStateMachine() {
        bool traffic_light_detected = this->computeDistanceToUpcomingTrafficLight()<1e6;

        this->state_machine->updateState(traffic_light_detected, this->traffic_light_state,
                                         this->getCurrentStoppingDistance(), this->current_speed,
                                         this->stop_distance_at_intersection);
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