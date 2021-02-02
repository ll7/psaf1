#include <pluginlib/class_list_macros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "plugin_global_planner.h"
#include <math.h>

#include <boost/foreach.hpp>
#include <std_msgs/String.h>

#define foreach BOOST_FOREACH

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(psaf_global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace psaf_global_planner {

    GlobalPlanner::GlobalPlanner(): costmap_ros(NULL) {

    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!init){
            statusPublisher = nodeHandle.advertise<std_msgs::String>("/psaf/status", 10);
            this->costmap_ros = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
            ROS_INFO("This planner is initialized");
            init = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
        std_msgs::String statusMsg;
        plan.clear();
        if(init && ros::ok()){
            // everything ok -> just trigger the local planner
            plan.push_back(start);
        } else {
            ROS_ERROR("Planner not initialized");
            statusMsg.data= "planner not initialized";
            statusPublisher.publish(statusMsg);
            plan.push_back(start);
            return false;
        }
    }
};