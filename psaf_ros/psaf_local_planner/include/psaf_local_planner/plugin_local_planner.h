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
#include <psaf_messages/XRoute.h>
#include <psaf_messages/XLanelet.h>
#include <psaf_local_planner/PsafLocalPlannerParameterConfig.h>
#include <base_local_planner/goal_functions.h>
#include <boost/algorithm/clamp.hpp>

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
            /**
             * Constructor: Nothing happens here, other than standard values are getting inited
             */
            PsafLocalPlanner();
            ~PsafLocalPlanner();
            
             /**
             * Constructs the local planner.
             * 
             * Overrides the nav_core::BaseLocalPlanner method; Called upon node creation
             * @param name: Name of the node in which this plugin runs
             * @param tf: ?
             * @param costmap_ros: Pointer to the costmap internal to move_base
             */
            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS *costmap_ros);
            

            /**
             * Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
             * 
             * Overrides the nav_core::BaseLocalPlanner method; Called every ROS Tick
             * Defacto main loop in the local planner
             * 
             * @param cmd_vel: Reference to the main velecity, to be used as the return parameter (no publish)
             */
            bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
            
            /**
             * Check if the goal pose has been achieved by the local planner.
             *
             * Overrides the nav_core::BaseLocalPlanner method; Called every ROS Tick
             *
             *  If this returns true the local planner gets shutdown
             */
            bool isGoalReached();
            
            /**
             * Set the plan that the local planner is following.
             * 
             * Overrides the nav_core::BaseLocalPlanner method; Called when the global planner publishes a global plan
             */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
        private:
            /**
             * Publishes the global plan to be displayed in RVIZ
             * 
             * @param path: Vector of poses that make up the plan
             */
            void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& path);

            /**
             * Callback from dynamic_reconfigure; used to set the local variables dynamically
             * 
             * @param config: Object containing all the changed parameters
             * @param level: Highest level of the changed parameter (see dyn_rec docs)
             */
            void reconfigure_callback(psaf_local_planner::PsafLocalPlannerParameterConfig &config, uint32_t level);

            /**
             * Deletes the points in the global plan that have been driven over
             * 
             * finds the closests point to the car atleast 2 meter ahead of the vehicle; deletes all before that one
             */
            void deleteOldPoints();
            
            /**
             * Computes the steering angle the car should take based on the current location of the car
             * 
             * @param target_location: Next point that should be targeted by the car; is affected by the lookahead
             * @param current_location: Current Location + Rotation of the car
             * @return angle of the wheels in rad
             */
            double compute_steering_angle(geometry_msgs::Pose target_location, geometry_msgs::Pose current_location);

            /**
             * Esitmates the curvature on the upcomuing road for the next [estimate_curvature_distance] distance
             * Used a 3 point method to calculate the radius of curves along the global plan
             * Finds the minimum radius in the next [estimate_curvature_distance] distance
             * 
             * Uses following formula for max speed in curves:
             *   max speed in curves: v <= sqrt(µ_haft * r * g)
             *   µ_haft ~= 0.8 - 1.0
             * 
             * Should be called in any case no matter the state; sets the base velocity
             * 
             * @param current_location: current pose of the car
             */
            void estimate_curvature_and_set_target_velocity(geometry_msgs::Pose current_location);

            /**
             * Helper function for calculating the radius of the curve on the global plan
             * 
             * third point is selected in the middle between first and last
             * @param first: index of the first point on the global plan of this curve
             * @param last: index of the last point on the global plan of this curve
             * @return radius of the circle in meters
             */
            double calculateRadius(unsigned int first, unsigned int last);

            /**
             * Checks the distance which is free on the global plan using the costmap
             * 
             * @return true if a obstacle was found in the bounds of the costmap
             * @param distance: return value of the distance forward 
             * @param relativeX: return value of relative X position at which the obstacle was found
             * @param relativeY: return value of relative Y position at which the obstacle was found
             */
            bool check_distance_forward(double& distance, double &relativeX, double &relativeY);
            
            /**
             * Send a raytrace along the costmap from the current position of the car
             * 
             * @param m_target_x: x position where to raytrace to in global map coordinates (in meters)
             * @param m_target_y: x position where to raytrace to in global map coordinates (in meters)
             * @param coll_x: x position where the collision happend on the costmap in global map coordinates (in meters)
             * @param coll_y: x position where the collision happend on the costmap in global map coordinates (in meters)
             * @return distance of the raytrace; returns infinity if there was no collision between those two points
             */
            double raytrace(double m_target_x, double m_target_y, double &coll_x, double &coll_y);

            /**
             * Sends out a bunch of raytraces around the car in a semi circle with the given angle
             * 
             * @param angle: the angle in rad around the car; circle is open to the back of the car if it isn't a full circle
             * @param distance: max distance that should be looked at by the raytrace
             * @param collisions: in case of collisions they are getting added to the vector
             */
            void raytraceSemiCircle(double angle, double distance, std::vector<RaytraceCollisionData> &collisions);

            /**
             * Function that checks for a slow car ahead using the slowdown of the own car
             * @param velocity_distance_diff: the difference between the max velocity and current that is caused by the slower car ahead
             * 
             * pubslishes obstacle positions using raytraces
             */
            void checkForSlowCar(double velocity_distance_diff);

            /**
             * Callback for the extened global plan by the global planner
             * 
             * @param msg: The message getting received
             */
            void globalPlanExtendedCallback(const psaf_messages::XRoute &msg);
            
            /** 
             * Finds the next target point along the global plan using the lookahead_factor and lookahead distance
             * 
             * @return pointer to the target point
             */
            geometry_msgs::PoseStamped& find_lookahead_target();

            /** The dynamic reconfigure server local for this node */
            dynamic_reconfigure::Server<psaf_local_planner::PsafLocalPlannerParameterConfig> *dyn_serv;

            /** The pointer to the costmap, updates automatically when the costmap changes */
            costmap_2d::Costmap2DROS* costmap_ros;

            /** Helper object from move_base */
            base_local_planner::LocalPlannerUtil planner_util;

            /** Helper object from move_base to find the current velocity of the vehicle */
            base_local_planner::OdometryHelperRos odom_helper;

            /** Publisher of the global plan for rviz */
            ros::Publisher g_plan_pub;

            /** Publisher for debug messages; e.g. arrows along path, obstacle bobbels */
            ros::Publisher debug_pub;

            /** Publisher for the obstacles; publishing causes a replanning */
            ros::Publisher obstacle_pub;

            /** ~~~ currently not used ~~~ */
            ros::Subscriber vel_sub;

            /** Subscriber for the extended global plan */
            ros::Subscriber global_plan_extended_sub;

            /** current pose of the vehicle; gets set  in computeVelocityCommands every ROS Tick */
            geometry_msgs::PoseStamped current_pose;

            /** Topic path of the odometrie */
            std::string odom_topic;
            
            /** The global plan that is only the poses */
            std::vector<geometry_msgs::PoseStamped> global_plan;

            /** The Route part of the xtenden route */
            std::vector<psaf_messages::XLanelet> global_route;

            /** ~~ unused ~~ was size of the local plan */
            int bufferSize;

            /** is set to true after the node is inited via ROS */ 
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

            /** Last time the obstacle message was published, allows repeated pubshling */
            ros::Time slow_car_last_published;

            /** ~~ planned ~~ current state of the car */
            LocalPlannerState state;

            /** Counter for the obstacle message to be always incrementing */
            unsigned int obstacle_msg_id_counter;

    };
};