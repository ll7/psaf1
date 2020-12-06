/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <std_msgs/String.h>

using std::string;

#ifndef PSAF_PLANNING_PLUGIN_GLOBAL_PLANNER_H
#define PSAF_PLANNING_PLUGIN_GLOBAL_PLANNER_H

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
        void pathCallback(const std_msgs::String::ConstPtr& msg);
        ros::Subscriber pathSubscriber;
        std::vector<geometry_msgs::PoseStamped> path;
        ros::NodeHandle nodeHandle;
        costmap_2d::Costmap2DROS* costmap_ros;
        bool init = false;
    };
};
#endif //PSAF_PLANNING_PLUGIN_GLOBAL_PLANNER_H
