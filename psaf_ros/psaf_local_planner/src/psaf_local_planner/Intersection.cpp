#include "psaf_local_planner/plugin_local_planner.h"


namespace psaf_local_planner {


    double PsafLocalPlanner::computeCheckDistance(){
        return std::max(1.5*getCurrentStoppingDistance(),15.);
    }

    double PsafLocalPlanner::computeDistanceToUpcomingTrafficLight() {

        if (global_route.size() > 0) {
            if (global_route[0].route_portion.size() > 1) {
                double remaining_way_on_lanelet =
                        this->distanceBetweenCenterLines(global_route[0].route_portion.front(),
                                                         global_route[0].route_portion.back());

                double check_radius = this->computeCheckDistance();
                // if the lanelet end is within the check radius we check the lanelet for a traffic light
                if (remaining_way_on_lanelet < check_radius) {
                    if(global_route[0].hasLight){
                        return remaining_way_on_lanelet;
                    }
                    return std::numeric_limits<double>::infinity();
                }
                // Check next lanelet if haven't seen a traffic light on the current one and
                // the end of the next lanelet is not far away
                if (global_route.size() > 1 && global_route[1].route_portion.size() > 1) {
                    double remaining_way_to_next_lanelet_end = remaining_way_on_lanelet +
                            this->distanceBetweenCenterLines(global_route[1].route_portion.front(),
                                                             global_route[1].route_portion.back());
                    if (remaining_way_to_next_lanelet_end > check_radius
                        && (remaining_way_on_lanelet < check_radius)) {
                        if(global_route[1].hasLight){
                            return remaining_way_to_next_lanelet_end;
                        }
                    }
                }
            }
        }
        return std::numeric_limits<double>::infinity();
    }

    double PsafLocalPlanner::computeDistanceToUpcomingStop() {

        if (global_route.size() > 0) {
            if (global_route[0].route_portion.size() > 1) {
                double remaining_way_on_lanelet =
                        this->distanceBetweenCenterLines(global_route[0].route_portion.front(),
                                                         global_route[0].route_portion.back());
                double check_radius = this->computeCheckDistance();
                // if the lanelet end is within the check radius we check the lanelet for a stop
                if (remaining_way_on_lanelet < check_radius) {
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
                    if (remaining_way_to_next_lanelet_end > check_radius
                        && (remaining_way_on_lanelet < check_radius)) {
                        if(global_route[1].hasStop){
                            return remaining_way_to_next_lanelet_end;
                        }
                        return std::numeric_limits<double>::infinity();
                    }
                }
            }
        }
        return std::numeric_limits<double>::infinity();
    }

    void PsafLocalPlanner::updateStateMachine() {
        bool traffic_light_detected = this->computeDistanceToUpcomingTrafficLight()<1e6;
        bool stop_detected = this->computeDistanceToUpcomingStop()<1e6;
        bool is_intersection_clear = false;
        // Without traffic rules we are less strict regarding a clear intersection
        if(this->respect_traffic_rules) {
            is_intersection_clear = this->costmap_raytracer.checkForNoMovement(M_PI, 25, 5);
        }else{
            is_intersection_clear = this->costmap_raytracer.checkForNoMovement(0.5*M_PI, 25, 5);
        }
        this->state_machine->updateState(traffic_light_detected, stop_detected,
                                         this->traffic_light_state,
                                         this->getCurrentStoppingDistance(), this->current_speed,
                                         this->stop_distance_at_intersection, is_intersection_clear, ros::Time::now().toSec());
        // Publish the new state
        this->publishCurrentStateForDebug();
    }

    double PsafLocalPlanner::getTargetVelIntersectionWithTrafficRules() {
        double target_vel = this->target_velocity;
        switch (this->state_machine->getState()) {
            case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
            case LocalPlannerState::STOP_GO:
                target_vel = getMaxVelocity();
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
            case LocalPlannerState::STOP_WILL_STOP:
                double velocity_distance_diff;
                double stop_distance;
                stop_distance = this->stop_distance_at_intersection;
                if(stop_distance >1e6){
                    stop_distance=0;
                }
                if (stop_distance < 3) {
                    velocity_distance_diff = this->target_velocity; // Drive very slow to stop line
                } else {
                    velocity_distance_diff = this->target_velocity - std::min(this->target_velocity, 25.0 / 18.0 * (-1 + std::sqrt(
                            1 + 4 * (stop_distance - 2))));
                }
                target_vel = this->target_velocity - velocity_distance_diff;
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
            case LocalPlannerState::STOP_NEAR:
                target_vel = target_velocity / 2;
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
            case LocalPlannerState::STOP_WAITING:
                target_vel = 0.0;
                break;
            default:
                target_vel = target_velocity;
        }
        return std::min(target_vel, target_velocity);
    }

    double PsafLocalPlanner::getTargetVelIntersectionWithoutTrafficRules() {
        double target_vel;
        switch (this->state_machine->getState()) {
            case LocalPlannerState::DRIVING:
                target_vel=this->target_velocity;
            case LocalPlannerState::STOP_GO:
                target_vel = getMaxVelocity();
                break;
            case LocalPlannerState::STOP_WILL_STOP:
                double velocity_distance_diff;
                double stop_distance;
                stop_distance = this->stop_distance_at_intersection;
                if(stop_distance >1e6){
                    stop_distance=0;
                }
                if (stop_distance < 3) {
                    velocity_distance_diff = this->target_velocity; // Drive very slow to stop line
                } else {
                    velocity_distance_diff = this->target_velocity - std::min(this->target_velocity, 25.0 / 18.0 * (-1 + std::sqrt(
                            1 + 4 * (stop_distance - 2))));
                }
                target_vel = this->target_velocity - velocity_distance_diff;
                break;
            case LocalPlannerState::STOP_NEAR:
                target_vel = target_velocity;
                break;
            case LocalPlannerState::STOP_WAITING:
                target_vel = 0.0;
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
            case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
            case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
            case LocalPlannerState::UNKNOWN:
            default:
                target_vel = this->target_velocity;

        }
        return std::min(target_vel, this->target_velocity);
    }


}