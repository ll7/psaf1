#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <string>
#include <filesystem>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>


namespace psaf_local_planner {
    class PsafLocalPlanner : public nav_core::BaseLocalPlanner {
        public:
            PsafLocalPlanner();
            ~PsafLocalPlanner();
                

            /**
             * Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
             */
            bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
            
            /**
             * Constructs the local planner.
             */ 
            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS *costmap_ros);
            
            /**
             * Check if the goal pose has been achieved by the local planner.
             */
            bool isGoalReached();
            
            /**
             * Set the plan that the local planner is following.
             */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
        private:
            ros::Publisher g_plan_pub;
            ros::Subscriber vel_sub;

    };
};