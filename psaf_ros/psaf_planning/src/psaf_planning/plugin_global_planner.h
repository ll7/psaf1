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

namespace psaf_global_planner {
    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
    public:
        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan
        );
    private:
        void loadPath(const std::string& filename);
        std::vector<geometry_msgs::PoseStamped> path;
        costmap_2d::Costmap2DROS* costmap_ros;
        bool init = false;
    };
};
