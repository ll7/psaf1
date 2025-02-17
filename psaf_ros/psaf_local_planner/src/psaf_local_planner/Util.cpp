#include "psaf_local_planner/plugin_local_planner.h"
#include <pluginlib/class_list_macros.h>


namespace psaf_local_planner
{

    void PsafLocalPlanner::reconfigureCallback(psaf_local_planner::PsafLocalPlannerParameterConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure Request");
    }

    /**
     * Check if the goal pose has been achieved by the local planner.
     */
    bool PsafLocalPlanner::isGoalReached()
    {
        return goal_reached && global_plan.size() <= 1;
    }

    /**
     * Set the plan that the local planner is following.
     */
    bool PsafLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        return true;
    }

    void PsafLocalPlanner::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (!path.empty())
            base_local_planner::publishPlan(path, g_plan_pub);
    }

    void PsafLocalPlanner::odometryCallback(const carla_msgs::CarlaEgoVehicleStatus &msg){
        this->current_speed = (double) msg.velocity * (msg.control.reverse?-1:1);
    }


    void PsafLocalPlanner::trafficSituationCallback(const psaf_messages::TrafficSituation::ConstPtr &msg) {

        if (msg->trafficLight.size() > 0) {
            this->traffic_light_state = msg->trafficLight.front();
        } else { // If no traffic light is detected set state to unknown
            psaf_messages::TrafficLight new_light;
            new_light.state = psaf_messages::TrafficLight::STATE_UNKNOWN;
            this->traffic_light_state = new_light;

        }
        // Distance to stop can be determined in two the distance to the end of the lanelet,
        // the distance to the traffic light on the right hand side or the distance to the stop line
        if(msg->distanceToStopLine<1e6){ // abstraction for infinity
            // Use the stop line distance only of it is plausible (= diff to computed value is <=10m)
            // and when we are inside a stop state
            double theoriaticalValue = this->computeDistanceToStoppingPointWithoutStopLine();
            if(std::abs(theoriaticalValue - msg->distanceToStopLine)>10 && this->state_machine->isInStopStates()) {
                this->stop_distance_at_intersection = theoriaticalValue;
            }else {
                this->stop_distance_at_intersection = msg->distanceToStopLine;
            }
        }else{
            this->stop_distance_at_intersection = this->computeDistanceToStoppingPointWithoutStopLine();
        }
//        ROS_WARN("Stop distance %f",this->stop_distance_at_intersection);
    }

    double PsafLocalPlanner::computeDistanceToStoppingPointWithoutStopLine() {
        if (state_machine->isInTrafficLightStates()){ // Use traffic light data only when in correct state
            // Use the traffic light distance if the perception already knows something
            if(this->traffic_light_state.state!=psaf_messages::TrafficLight::STATE_UNKNOWN){
                // if the traffic_light is on the right hand side we want to stop 5 meters in front of it
                if(this->traffic_light_state.x>0.6 && traffic_light_state.distance<15.){
                    return std::max(this->traffic_light_state.distance-5.,0.);
                }else{
                    // else the traffic light is on the other side of the intersection (american style)
                    // -> keep 30 meters as distance
                    double distance_to_traffic_light_lanelet = this->computeDistanceToUpcomingLaneletAttribute(&hasLaneletTrafficLight);
                    double distance_to_traffic_light_perception = std::max(this->traffic_light_state.distance-30.,0.);
                    // Respect lanelet data for american traffic lights because the distance we want to keep depends
                    // on the road size -> take the min value
                    // only use the lanelet data if it has data about a tl nearby
                    if(distance_to_traffic_light_lanelet<1e6 &&
                          std::abs(distance_to_traffic_light_lanelet-distance_to_traffic_light_perception)<40){
                        return std::max(distance_to_traffic_light_lanelet-2,distance_to_traffic_light_perception);
                    }
                    return distance_to_traffic_light_perception;
                }
            }
            // if we don't have any other information we use the map data minus 1 meter as safety distance
            double distance_to_traffic_light = this->computeDistanceToUpcomingLaneletAttribute(&hasLaneletTrafficLight);
            if( distance_to_traffic_light <1e6){
                return std::max((distance_to_traffic_light-1),0.0);
            }
            // No success go to fallback return

        }else if (state_machine->isInStopStates()) {
            // because we don't have any other information we use the map data
            double distance_to_stop = this->computeDistanceToUpcomingLaneletAttribute(&hasLaneletStop);
            // If we drive without traffic rules -> traffic lights equals stops -> get distance of a traffic light if there is one
            if(!respect_traffic_rules){
                distance_to_stop = std::min(distance_to_stop,this->computeDistanceToUpcomingLaneletAttribute(&hasLaneletTrafficLight));
            }
            if(distance_to_stop < 1e6){
                return std::max((distance_to_stop),0.0);
            }
            // No success go to fallback return

        }
        // if we didn't find a way to guess the distance let's return infinity because the stopping point must be far away
        return std::numeric_limits<double>::infinity(); // "to infinity and beyond" ~ buzz lightyear
    }

    double PsafLocalPlanner::distanceBetweenCenterLines(psaf_messages::CenterLineExtended first, psaf_messages::CenterLineExtended second){
        return second.distance-first.distance;
    }


    void PsafLocalPlanner::publishCurrentStateForDebug() {
        std_msgs::String msg;
        msg.data = this->state_machine->getTextRepresentation();

        this->debug_state_pub.publish(msg);
    }

    double computeSpeedToStopInXMeters(double wishedSpeed, double stoppingDistance){
        if (stoppingDistance < 3) {
            return 0; // Drive very slow to stop line
        } else {
            return std::min(wishedSpeed, 25.0 / 18.0 * (-1 + std::sqrt(
                    1 + 4 * (stoppingDistance - 1))));
        }
    }

}
