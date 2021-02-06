#pragma once

// General
#include <string>
#include <filesystem>
#include <assert.h>

// ros
#include <ros/ros.h>


// Messages
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>

// Geometry messages
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// Tf2
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// Nav / move stack
#include <nav_core/base_local_planner.h>

#include <base_local_planner/line_iterator.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>


// internal
#include <psaf_messages/Obstacle.h>
#include <psaf_messages/XRoute.h>
#include <psaf_messages/XLanelet.h>
#include <psaf_messages/TrafficLight.h>
#include <psaf_messages/TrafficSituation.h>
#include <psaf_local_planner/PsafLocalPlannerParameterConfig.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/costmap_model.h>
#include <psaf_local_planner/state_machine.h>
#include "psaf_local_planner/costmap_raytracer.h"

// external non ros
#include <boost/algorithm/clamp.hpp>
#include <assert.h>


namespace psaf_local_planner {

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

            // circle fraction area around car in which possible obstacles are published
            static constexpr double OBSTACLE_AREA = (M_PI * 3/2);
            // radius of the circle fraction area around car in which possible obstacles are published
            static constexpr double OBSTACLE_AREA_RADIUS =  30.0;
            // threshold of velocity difference when a car in front is counted as too slow
            static constexpr double VEL_DIFF_THRESHOLD = 5.0;
            // number of times a obstacle is counted as too slow until it gets published as obstacle
            static constexpr int NUM_SLOW_CAR_PUB = 30;
            // number at which obstacle counts as lost when decreasing obstacle count
            static constexpr int NUM_SLOW_CAR_DEL = 10;
            // distance in front of intersection in which obstacles should not be published
            static constexpr double MIN_DISTANCE_INTERSECTION = 30.0;
            // threshold for min costmap value
            static constexpr unsigned char COSTMAP_THRESHOLD = 128;

            // How many points the delete old points function should look ahead to find the closest point
            static constexpr int MAX_DELETE_OLD_POINTS_LOOKAHEAD = 100;
            
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
            void reconfigureCallback(psaf_local_planner::PsafLocalPlannerParameterConfig &config, uint32_t level);

            /**
             * Callback from perception_evaluation; used to receive upcoming traffic lights and stop lines
             *
             * @param msg: TrafficSituation-Message
             */
            void trafficSituationCallback(const psaf_messages::TrafficSituation::ConstPtr &msg);

            /**
             * Callback for carla vehicle status messages.
             * Called by the subscriber
             * @param msg: CarlaEgoVehicleStatus the messages
             */
            void odometryCallback(const carla_msgs::CarlaEgoVehicleStatus &msg);

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
            double computeSteeringAngle(geometry_msgs::Pose target_location, geometry_msgs::Pose current_location);

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
            double estimateCurvatureAndSetTargetVelocity();

            /**
             * Retruns target speed with check for obstacles in the way
             *
             * @returns target_vel: target velocity for driving with vehicles
             */
            double getTargetVelDriving();


            /**
             * reads the max velocity from the current nearest point of the xroute to the car
             */
            double getMaxVelocity();

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
            bool checkDistanceForward(double& distance, double &relative_x, double &relative_y);

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
             * Calculates the max distance to the next intersection
             */
            double getDistanceToIntersection();

            /** 

            /**
             * Calculates suitable target speed according to current traffic light state
             *
             * @returns target_vel: current target velocity with attention to traffic lights
             */
            double getTargetVelIntersection();

            /**
             * Calculates the current stopping distance if the car makes a full braking
             * @returns the stopping distance in meters
             */
            double getCurrentStoppingDistance();

            /**
             * Update the current state of the state machine
             * Called in computeVelocityCommands to keep the state up to date
             */
            void updateStateMachine();

            /**
             * Computes the distance to traffic light on lanelet if the lanelet before the upcoming intersection
             * has a traffic light that affects the car.
             * Checks the current lanelet and the next lanelet if the current one is too short
             * @return the distance to the traffic light. Infinity if the is none.
             */
            double computeDistanceToUpcomingTrafficLight();

            /**
            * Computes the distance to stop mark/sign on lanelet if the lanelet before the upcoming intersection
             * has a stop sign or a stop mark that affects the car.
            * Checks the current lanelet and the next lanelet if the current one is too short
            * @return the distance to the stop. Infinity if the is none.
            */
            double computeDistanceToUpcomingStop();

            /**
             * Computes the euclidean 2d distance between two center lines.
             * @param first the first center line
             * @param second the second center line
             * @return the euclidean distance in 2d space
             */
            double distanceBetweenCenterLines(psaf_messages::CenterLineExtended first, psaf_messages::CenterLineExtended second);

            /**
             * Finds the next target point along the global plan using the lookahead_factor and lookahead distance
             *
             * @return pose of the target point
             */
            geometry_msgs::Pose findLookaheadTarget(psaf_messages::XLanelet &lanelet_out, psaf_messages::CenterLineExtended &center_point_out);

            /**
             * Cacluclates the distance to the next upcoming lanechange
             * 
             * @param direction[out]: +1 for right side; -1 for left side
             * @return distance to the direction
             */
            double getDistanceToLaneChange(double compute_direction_threshold);

            double checkLaneChangeFree();


            /** Helper object for raytracing */
            psaf_local_planner::CostmapRaytracer costmap_raytracer;

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

            /** Subscriber for the traffic situation message from perception_evaluation */
            ros::Subscriber traffic_situation_sub;

            /** Subscriber for vehicle status updates */
            ros::Subscriber vehicle_status_sub;

            /** Subscriber for the extended global plan */
            ros::Subscriber global_plan_extended_sub;

            /** current pose of the vehicle; gets set  in computeVelocityCommands every ROS Tick */
            geometry_msgs::PoseStamped current_pose;

            std::vector<geometry_msgs::PoseStamped> global_plan;

            /** The Route part of the xtenden route */
            std::vector<psaf_messages::XLanelet> global_route;

            /** How many points have been deleted from the global path*/
            unsigned int deleted_points;

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

            /** Last time a slow car message was published, allows repeated pubshling */
            ros::Time slow_car_last_published;

            /** Last time the obstacle message was published, prevents sending too often */
            ros::Time obstacle_last_published;

            /** */
            unsigned int slow_car_last_published_deleted_points;

            /** ~~ planned ~~ current state of the car */
            LocalPlannerState state;
            
            /** The state machine */
            LocalPlannerStateMachine* state_machine;

            /** Counter for the obstacle message to be always incrementing */
            unsigned int obstacle_msg_id_counter;

            /** Factor by which the new route can be slower as the old route and still be accepted as new route */
            double duration_factor;

            /** Factor by which the new route can be longer as the old route and still be accepted as new route */
            double distance_factor;

            /** sets a flag if traffic rules have to be obeyed or not */
            bool respect_traffic_rules;


            /** Detected traffic light state -> null means that there is no Traffic light state*/
            psaf_messages::TrafficLight traffic_light_state;

            /** Distance to detected stop line */
            double stop_distance_at_intersection;

            /**
             * The current speed of the car
             */
            double current_speed;

            /** max number of points used at beginning and end of lanelet when smoothing lanechanges */
            unsigned long max_points_smoothing;

            /**variable to determine whether the next lanechange is to the right(1) or to the left(-1)  */
            int lane_change_direction;
            /**variable to determine whether the direction of the lext lane change was already calcualted  */
            bool lane_change_direction_calculated;

    };
};