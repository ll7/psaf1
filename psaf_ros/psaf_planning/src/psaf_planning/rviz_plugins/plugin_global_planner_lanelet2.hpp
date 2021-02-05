#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <string>
#include <filesystem>

namespace psaf_global_planner {
    class GlobalPlannerLanelet2 : public nav_core::BaseGlobalPlanner {
    public:
        GlobalPlannerLanelet2();
        GlobalPlannerLanelet2(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan
        );
    private:
        bool nearly_equal(float a, float b, float epsilon = 128 * FLT_EPSILON, float relth = FLT_MIN);
        bool comparePosition(geometry_msgs::Pose a, geometry_msgs::Pose b);
        bool loadPath(const std::string& filename);

        ros::Publisher statusPublisher;
        ros::NodeHandle nodeHandle;

        std::vector<geometry_msgs::PoseStamped> path;
        costmap_2d::Costmap2DROS* costmap_ros;
        bool init = false;
    };
};
