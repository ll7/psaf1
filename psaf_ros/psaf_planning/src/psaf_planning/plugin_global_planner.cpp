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

    bool GlobalPlanner::loadPath(const std::string& filename) {
        try{
            // Read from File to pathMsg
            ROS_INFO("Loading path from: %s", filename.c_str());
            nav_msgs::Path pathMsg;
            rosbag::Bag bag;
            bag.open(filename, rosbag::bagmode::Read);

            std::vector<std::string> topics;
            topics.push_back(std::string("Path"));

            rosbag::View view(bag, rosbag::TopicQuery(topics));
            foreach(rosbag::MessageInstance const m, view)
            {
                nav_msgs::PathPtr s = m.instantiate<nav_msgs::Path>();
                if (s != NULL)
                    path = s->poses;
            }
            bag.close();
            return true;
        } catch (...) {
            return false;
        }
    }

    bool GlobalPlanner::nearly_equal( float a, float b, float epsilon, float relth) {
        assert(std::numeric_limits<float>::epsilon() <= epsilon);
        assert(epsilon < 1.f);

        if (a == b) return true;

        auto diff = std::abs(a-b);
        auto norm = std::min((std::abs(a) + std::abs(b)), std::numeric_limits<float>::max());
        return diff < std::max(relth, epsilon * norm);
    }
    bool GlobalPlanner::comparePosition(geometry_msgs::Pose a, geometry_msgs::Pose b) {
        return a.position.x == b.position.x && a.position.y == b.position.y && a.position.z == b.position.z;
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
        std_msgs::String statusMsg;
        plan.clear();
        if(init && ros::ok()){
            std::string filename;
            for (const auto & entry : std::filesystem::directory_iterator("/tmp"))
                if(entry.path().string().find(".path") != std::string::npos) {
                    filename = entry.path().string();
                    break;
                }
            if(!loadPath(filename)) {//get path from file
                plan.push_back(start);
                return true;
            }
            if(path.size() > 0) {
                if(comparePosition(path.back().pose, goal.pose)) {
                    plan = path; //move the path to the plan vector witch is used by the move_base to follow the path
                    ROS_INFO("Path size: %d vs %d(msg)", plan.size(), path.size());
                    ROS_INFO("Global planner was successful");
                    statusMsg.data= "Global planner was successful";
                    statusPublisher.publish(statusMsg);
                    return true;
                } else {
                    plan.push_back(start);
                    ROS_ERROR("Path start and end positions don't match");
                    statusMsg.data= "Path start and end positions don't match";
                    statusPublisher.publish(statusMsg);
                    return false;
                }
            } else {
                ROS_ERROR("No path received, execute the path planner package and try again");
                statusMsg.data= "No path received, execute the path planner package and try again";
                statusPublisher.publish(statusMsg);
                plan.push_back(start);
                return false;
            }
        } else {
            ROS_ERROR("Planner not initialized");
            statusMsg.data= "planner not initialized";
            statusPublisher.publish(statusMsg);
            plan.push_back(start);
            return false;
        }
    }
};