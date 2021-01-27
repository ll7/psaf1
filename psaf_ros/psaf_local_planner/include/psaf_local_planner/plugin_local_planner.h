#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <string>
#include <filesystem>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/local_planner_util.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <base_local_planner/line_iterator.h>
#include <psaf_messages/Obstacle.h>
#include "psaf_local_planner/PsafLocalPlannerParameterConfig.h"


namespace psaf_local_planner {

    enum LocalPlannerState {
        /** 
         * Car has stopped 
         * --> DRIVING [on global plan published] 
         */
        STOPPED, 

        /**
         * Car has reached final destination
         * --> shutdown
         */
        GOAL_REACHED, 

        /**
         * Car is driving on normal, straight lane
         * --> DRIVING_CURVATURE [on curvature ahead less than x meters with angle over y]
         * --> DRIVING_INTERSECTION [on driving over a intersection]
         * --> GOAL_REACHED [on reaching goal reached]
         */
        DRIVING, 
        /**
         * Car is driving in or ahead of a curvature
         * --> DRIVING [on: car has left the curvature and is back on a straight track]
         * --> DRIVING_INTERSECTION_AHEAD [on: car has left the curvature and has a intersection ahead of it]
         */
        DRIVING_CURVATURE, 

        /**
         * 
         */
        DRIVING_INTERSECTION_AHEAD,
        DRIVING_INTERSECTION, 
        STOPPED_INTERSECTION_REDLIGHT,
        STOPPED_INTERSECTION_STOP_SIGN,
    };

    class RaytraceCollisionData {
        public:
            RaytraceCollisionData(double x, double y, double angle, double distance);
            double x, y, angle, distance;
    };


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
            void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& path);
            void reconfigure_callback(psaf_local_planner::PsafLocalPlannerParameterConfig &config, uint32_t level);

            void deleteOldPoints();
            
            
            double compute_steering_angle(geometry_msgs::Pose target_location, geometry_msgs::Pose current_location);
            void estimate_curvature_and_set_target_velocity(geometry_msgs::Pose current_location);
            double calculateRadius(unsigned int first, unsigned int last);

            bool check_distance_forward(double& distance, double &relativeX, double &relativeY);

            double raytrace(double m_target_x, double m_target_y, double &coll_x, double &coll_y);
            void raytraceSemiCircle(double angle, double distance, std::vector<RaytraceCollisionData> &collisions);
            void checkForSlowCar(double velocity_distance_diff);
            void globalPlanExtendedCallback(const geometry_msgs::Twist &msg);
            
            dynamic_reconfigure::Server<psaf_local_planner::PsafLocalPlannerParameterConfig> *dyn_serv;

            geometry_msgs::PoseStamped& find_lookahead_target();

            costmap_2d::Costmap2DROS* costmap_ros;
            base_local_planner::LocalPlannerUtil planner_util;

            ros::Publisher g_plan_pub;
            ros::Publisher debug_pub;
            ros::Publisher obstacle_pub;
            ros::Subscriber vel_sub;
            ros::Subscriber global_plan_extended_sub;

            geometry_msgs::PoseStamped current_pose;
            base_local_planner::OdometryHelperRos odom_helper;
            std::string odom_topic;
            

            std::vector<geometry_msgs::PoseStamped> global_plan;
            int bufferSize;

            bool initialized;

            /** Actual Max velocity that is allowed to be driven */
            double max_velocity;
            
            /** min speed that the car should drive at even through sharp angles */
            double min_velocity;

            /** the velocity that is targeted; influenced by the angle of the road etc. */
            double target_velocity;
            
            /** All points closer than this value are getting rejected */
            double closest_point_local_plan;
            
            /** floor (target_velocity / lookahead_factor) == index of the point to the use in the local planner list; lower is more points */
            double lookahead_factor;

            /** set to true when the goal is reached*/
            bool goal_reached;

            /** The distance for which it should look for a curvature*/
            double estimate_curvature_distance;

            /** The max distance it should check for a collision*/
            double check_collision_max_distance;

            /** counts the iterations the car has been detected as slow before us */
            int slow_car_ahead_counter;

            /** sets a flag when the slow car and obstacles have been published */ 
            bool slow_car_ahead_published;

            /** */
            ros::Time slow_car_last_published;

            LocalPlannerState state;

            unsigned int obstacle_msg_id_counter;

    };
};