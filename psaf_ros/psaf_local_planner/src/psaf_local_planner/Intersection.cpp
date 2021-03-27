#include "psaf_local_planner/plugin_local_planner.h"


namespace psaf_local_planner {


    double PsafLocalPlanner::getCheckDistanceForLanelets() {
        return std::max(1.5 * getCurrentStoppingDistance(), MIN_CHECK_RADIUS_FOR_TRAFFIC_ELEMENTS);
    }

    bool hasLaneletStop(psaf_messages::XLanelet lanelet) {
        return lanelet.hasStop;
    }

    bool hasLaneletTrafficLight(psaf_messages::XLanelet lanelet) {
        return lanelet.hasLight;
    }

    double PsafLocalPlanner::computeDistanceToUpcomingLaneletAttribute(
            bool (*attributeCheckFunction)(psaf_messages::XLanelet)) {

        if (global_route.size() >= 2) { // at least two lanelets in list -> 1 lanelet isn't important
            // -> goal lanelet and don't care about a traffic light or stop
            if (global_route[0].route_portion.size() > 1) {
                double remaining_way_on_lanelet =
                        this->distanceBetweenCenterLines(global_route[0].route_portion.front(),
                                                         global_route[0].route_portion.back());

                // if the lanelet end is outside the check radius we check the lanelet for a stop
                if (remaining_way_on_lanelet > this->LANELET_CHECK_RADIUS) {
                    return std::numeric_limits<double>::infinity();
                }
                if (attributeCheckFunction(global_route[0])) {
                    return remaining_way_on_lanelet;
                }
                // Check next lanelet
                if (global_route[1].route_portion.size() > 1) {
                    double remaining_way_to_next_lanelet_end = remaining_way_on_lanelet +
                                                               this->distanceBetweenCenterLines(
                                                                       global_route[1].route_portion.front(),
                                                                       global_route[1].route_portion.back());
                    // If the next lanelet end is inside the check radius
                    if (remaining_way_to_next_lanelet_end < this->LANELET_CHECK_RADIUS) {
                        if (attributeCheckFunction(global_route[1])) {
                            return remaining_way_to_next_lanelet_end;
                        }
                    }
                }
            }
        }
        return std::numeric_limits<double>::infinity();
    }

    void PsafLocalPlanner::updateStateMachine() {
        // A traffic light is detected if it is within our check distance on our path
        bool traffic_light_detected = this->computeDistanceToUpcomingLaneletAttribute(&hasLaneletTrafficLight) <=
                                      this->getCheckDistanceForLanelets();
        // A stop is detected if it is within our check distance on our path
        bool stop_detected = this->computeDistanceToUpcomingLaneletAttribute(&hasLaneletStop) <=
                             this->getCheckDistanceForLanelets();
        // whether the intersection is clear
        bool is_intersection_clear = false;
        // Without traffic rules we are less strict regarding a clear intersection
        if (this->state_machine->getState() == LocalPlannerState::STOP_WAITING) {
            if (this->respect_traffic_rules) {
                is_intersection_clear = this->costmap_raytracer.checkForNoMovement(0.5 * M_PI, 22, 5);
            } else {
                is_intersection_clear = this->costmap_raytracer.checkForNoMovement(0.5 * M_PI, 18, 5);
            }
        }
        this->state_machine->updateState(traffic_light_detected, stop_detected,
                                         this->traffic_light_state,
                                         this->getCurrentStoppingDistance(), this->current_speed,
                                         this->stop_distance_at_intersection, is_intersection_clear,
                                         ros::Time::now().toSec());
        // Publish the new state
        this->publishCurrentStateForDebug();
    }

    double PsafLocalPlanner::getTargetVelIntersectionWithTrafficRules() {
        double target_vel = this->target_velocity;
        switch (this->state_machine->getState()) {
            case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
                // If the traffic light is within out stopping distance but greater 15m
                // and we don't have any information about the traffic state (= indicate by this global planner state),
                // reduce the speed to be slow enough to be able to stop at the traffic light when we know more
                if(this->stop_distance_at_intersection > MIN_CHECK_RADIUS_FOR_TRAFFIC_ELEMENTS
                        &&this->stop_distance_at_intersection<getCurrentStoppingDistance()){
                    // check if that's not the last lanelet else ignore this use case
                    if(this->global_route.size() == 1) {
                        ROS_DEBUG("Reduce speed because near traffic light and no knowledge about the traffic light state");
                        target_vel = computeSpeedToStopInXMeters(this->target_velocity,
                                                                 this->stop_distance_at_intersection);
                    }
                }
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
            case LocalPlannerState::STOP_GO:
                // We can go
                target_vel = getMaxVelocity();
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
            case LocalPlannerState::STOP_WILL_STOP:
                // Stop at stop line
                double stop_distance;
                stop_distance = this->stop_distance_at_intersection;
                if (stop_distance >  1e6) {
                    // Stop because we should see an stop line but we have no data -> emergency brake
                    ROS_WARN("Emergency brake due to missing information about distance to stop line but state"
                              "forces the car to stop at a stop line -> for safety brake now");
                    stop_distance = 0;
                }
                target_vel = computeSpeedToStopInXMeters(this->target_velocity,stop_distance);
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
                // Slow down and let's hope that the traffic light will turn green
                target_vel = target_velocity * 0.5;
                break;
            case LocalPlannerState::STOP_NEAR:
                // Do absolute nothing -> keep driving with the same speed
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WAITING:
            case LocalPlannerState::STOP_WAITING:
                // have to do wait
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
                // This is a duplicate code but keep it for a better understanding
                target_vel = this->target_velocity;
            case LocalPlannerState::STOP_GO:
                target_vel = getMaxVelocity();
                break;
            case LocalPlannerState::STOP_WILL_STOP:
                target_vel = computeSpeedToStopInXMeters(this->target_velocity,this->stop_distance_at_intersection);
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