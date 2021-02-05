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
        } else {
            psaf_messages::TrafficLight new_light;
            new_light.state = psaf_messages::TrafficLight::STATE_UNKNOWN;
            this->traffic_light_state = new_light;

        }
        this->stop_distance_at_intersection = msg->distanceToStopLine;
    }
}
