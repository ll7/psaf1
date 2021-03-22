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
            // Use the stop line distance
            this->stop_distance_at_intersection = msg->distanceToStopLine;
        }else{
            this->stop_distance_at_intersection = this->computeDistanceToStoppingPointWithoutStopLine();
        }
        ROS_WARN("Stop distance %f",this->stop_distance_at_intersection);
    }

    double PsafLocalPlanner::computeDistanceToStoppingPointWithoutStopLine() {
        if (state_machine->isInTrafficLightStates()){ // Use traffic light data only when in correct state
            // Use the traffic light distance if the perception already knows something
            if(this->traffic_light_state.state!=psaf_messages::TrafficLight::STATE_UNKNOWN){
                // if the traffic light is on the other side of the intersection (american style)
                if(this->traffic_light_state.x<0.8 && this->traffic_light_state.y<0.32 && traffic_light_state.distance>15.){
                    // we keep 20 meters as distance
                    return std::max(this->traffic_light_state.distance-20.,0.);
                }else{
                    // else the traffic_light is on the right hand side we want to stop 5 meters in front of it
                    return std::max(this->traffic_light_state.distance-5.,0.);
                }
            } else{ // if we don't have any other information we use the map data minus 10 meters as safety distance
                    double distance_to_traffic_light = this->computeDistanceToUpcomingLaneletAttribute(&hasLaneletTrafficLight);
                    if( distance_to_traffic_light <1e6){
                        return std::max((distance_to_traffic_light-10),0.0);
                    }
                    // No success go to fallback return
                }
        }else if (state_machine->isInStopStates()) {
            // because we don't have any other information we use the map data
            double distance_to_stop = this->computeDistanceToUpcomingLaneletAttribute(&hasLaneletStop);
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


}
