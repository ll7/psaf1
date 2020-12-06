#include <pluginlib/class_list_macros.h>
#include "plugin_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(psaf_global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace psaf_global_planner {

    GlobalPlanner::GlobalPlanner (){}

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
        pathSubscriber = nodeHandle.subscribe("/pasf/path/get", 1000, &psaf_global_planner::GlobalPlanner::pathCallback, this);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!init){
            this->costmap_ros = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
            ROS_WARN("This planner is intialized");
            init = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    void GlobalPlanner::pathCallback(const std_msgs::String::ConstPtr& msg) {
        //path = msg->poses;
        ROS_INFO("I heard: [%s]", msg->data.c_str());
        ROS_INFO("Recieved path");
        //ROS_INFO("Path size:%d vs %d (msg)", path.size(), msg->poses.size());
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        if(init){
            if(path.size() > 0) {
                if(path.at(0) == start && path.back() ==goal) {
                    plan = path;
                    ROS_INFO("Global planner was successful");
                    return true;
                } else {
                    plan.push_back(start);
                    ROS_ERROR("Path start and end positions don't match");
                }
            } else {
                //ROS_ERROR("No path recieved, please call /pasf/path/get and try again");
                plan.push_back(start);
                return false;
            }
        }else {
            ROS_ERROR("Planner not initialized");
            plan.push_back(start);
            return false;
        }
    }
};