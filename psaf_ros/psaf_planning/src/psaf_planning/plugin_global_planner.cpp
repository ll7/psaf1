#include <pluginlib/class_list_macros.h>
#include "plugin_global_planner.h"

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
            this->costmap_ros = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
            ROS_INFO("This planner is initialized");
            init = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    void GlobalPlanner::loadPath(const std::string& filename) {
        // Read from File to pathMsg
        ROS_INFO("Loading path from: /tmp/%s", filename.c_str());
        nav_msgs::Path pathMsg;

        std::ifstream ifs("/tmp/"+filename, std::ios::in|std::ios::binary);
        ifs.seekg (0, std::ios::end);
        std::streampos end = ifs.tellg();
        ifs.seekg (0, std::ios::beg);
        std::streampos begin = ifs.tellg();

        uint32_t file_size = end-begin;
        boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
        ifs.read((char*) ibuffer.get(), file_size);
        ros::serialization::IStream istream(ibuffer.get(), file_size);
        ros::serialization::deserialize(istream, pathMsg);
        path = pathMsg.poses;
        ROS_INFO("Path loaded with length: %d", (int)path.size());
        ifs.close();
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
        if(init && ros::ok()){
            loadPath("path.path"); //get path from file
            if(path.size() > 0) {
                if(path.at(0) == start && path.back() == goal) {
                    plan = path; //move the path to the plan vector witch is used by the move_base to follow the path
                    ROS_INFO("Global planner was successful");
                    return true;
                } else {
                    plan.push_back(start);
                    ROS_ERROR("Path start and end positions don't match");
                    return false;
                }
            } else {
                ROS_ERROR("No path received, execute the path pLaner package and try again");
                plan.push_back(start);
                return false;
            }
        } else {
            ROS_ERROR("Planner not initialized");
            plan.push_back(start);
            return false;
        }
    }
};