#include "psaf_local_planner/plugin_local_planner.h"


namespace psaf_local_planner
{

    void PsafLocalPlanner::trafficSituationCallback(const psaf_messages::TrafficSituation::ConstPtr &msg)
    {
        detected_traffic_lights = msg->trafficLight;
        if (msg->distanceToStopLine < 100) {
            stop_line_distance = msg->distanceToStopLine;
            //this->state_machine->setState(LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP);
            ROS_INFO("StopLine detected %f", stop_line_distance);
        }

    }

    double PsafLocalPlanner::getTargetVelIntersection()
    {
        double target_vel = target_velocity;
        switch(this->state_machine->getState())
        {
            case LocalPlannerState::TRAFFIC_LIGHT_NEAR:
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_GO:
                target_vel = getMaxVelocity();
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_WILL_STOP:
                double velocity_distance_diff;
                if (stop_line_distance < 3)
                {
                    velocity_distance_diff = target_velocity;
                } else {
                    velocity_distance_diff = target_velocity - std::min(target_velocity, 25.0/18.0 * (-1 + std::sqrt(1 + 4 * (stop_line_distance - 2))));
                }
                target_vel = target_velocity - velocity_distance_diff;
                break;
            case LocalPlannerState::TRAFFIC_LIGHT_SLOW_DOWN:
                target_vel = target_velocity/2;
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