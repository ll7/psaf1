#pragma once

// General
#include <string>
#include <filesystem>
#include <cassert>

// ros
#include <ros/ros.h>


// Messages
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

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

// Constants
// Check min radius for traffic elements in global plan
#define MIN_CHECK_RADIUS_FOR_TRAFFIC_ELEMENTS 15.0
// minimal distance to center line to prevent attribute check during an u-turn
#define MIN_DISTANCE_TO_CENTERLINE_FOR_ATTRIBUTE_CHECK 2.0


// Ensure that we work with the correct standard
// see https://stackoverflow.com/questions/41160276/does-c-stds-inf-behave-exactly-as-common-sensical-infinity
static_assert(std::numeric_limits<double>::is_iec559, "The numeric standard doesn't fit our requirements!");

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
            static constexpr double MIN_DISTANCE_INTERSECTION = 20.0;
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
             * Returns target speed with check for obstacles in the way
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
             * Calculates suitable target speed according to right of way situation at the intersection
             *
             * @returns target_vel: current target speed according to right of way situation at the intersection
             */
            double getTargetVelIntersectionWithTrafficRules();

            /**
            * Calculates suitable target speed too pass the intersection as fast as possible but it ignores the right of way
            *
            * @returns target_vel: current target speed according to pass the intersectionsssss
            */
            double getTargetVelIntersectionWithoutTrafficRules();

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
             * Returns the distance within the car checks for traffic light and stop signs/marks.
             * @return the distance within the car checks for traffic light and stop signs/marks.
             */
            double getCheckDistanceForLanelets();


            /**
            * Computes the distance to attribute on a lanelet if the lanelet before the upcoming intersection
            * fulfills the attribute check.
            * Checks the current lanelet and the next lanelet if the current one is too short
            * @return the distance to the attribute. Infinity if there is none.
            */
            double computeDistanceToUpcomingLaneletAttribute(bool (*attributeCheck)(psaf_messages::XLanelet));

            /**
             * Helper function to compute the distance to a stop line if no stop line is detected
             * If the state tells us that a traffic light is near we use the detected traffic light as indicator or
             * the lanelet data about the remaining lane length
             * If the state tells us that a stop sign/marking is near we use the the lanelet data about the remaining lane length to the stop sign / mark
             * @return the computed distance as described above
             */
            double computeDistanceToStoppingPointWithoutStopLine();

            /**
             * Computes the euclidean 2d distance between two center lines.
             * @param first the first center line
             * @param second the second center line
             * @return the euclidean distance in 2d space
             */
            double distanceBetweenCenterLines(psaf_messages::CenterLineExtended first, psaf_messages::CenterLineExtended second);

            /**
             * Publishes the current state as a string for debugging
             */
            void publishCurrentStateForDebug();

            /**
             * Finds the next target point along the global plan using the lookahead_factor and lookahead distance
             *
             * @param lanelet_out: Lanelet where the target point is on
             * @param center_point_out: target point in the XRoute format
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

            /**
             * Checks if area of the lane change is free and return speed accordingly
             * 
             * @return new max target velocity
             */ 
            double checkLaneChangeFree();

            /**
             * Calculates how fast we should drive to be able to stop in time at before the given distance
             * 
             * uses formula for Anhalteweg (solved for velocity instead of distance)
             * https://www.bussgeldkatalog.org/anhalteweg/
             */
            double getTargetVelocityForDistance(double distance);


            /**
             * Helper function to publish data like stopsigns etc to rviz
             */
            void publishAdditionalInfoToRviz();


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
            ros::Publisher debug_marker_pub;

            /** Publisher for debug messages about the state machine */
            ros::Publisher debug_state_pub;

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

            /** Exponential factor for the lookahead */
            double lookahead_factor_exp;
            
            /** Additive lookahead factor not depending on the speed*/
            double lookahead_factor_const_additive;

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

            /**
             * "View" radius that defines the distance within the lanelets will be checked for stops and traffics lights.
             */
            const double LANELET_CHECK_RADIUS = 100;

            /**
             * Map of lanechanges; key is id of the lanelet; value is +1 for right, -1 for left
             */
            std::map<int, int> lanechange_direction_map;

    };

    /**
     * Checks whether the given lanelet contains a stop at its end.
     * This is used to be passed as a function pointer to reduce duplicate code in computeDistanceToUpcomingLaneletAttribute
     * @param lanelet the lanelet that will be check
     * @return whether the lanelet has a stop sign/mark
     */
    bool hasLaneletStop(psaf_messages::XLanelet lanelet);

    /**
    * Checks whether the given lanelet contains a stop at its end.
    * This is used to be passed as a function pointer to reduce duplicate code in computeDistanceToUpcomingLaneletAttribute
    * @param lanelet the lanelet that will be check
    * @return whether the lanelet has a stop sign/mark
    */
    bool hasLaneletTrafficLight(psaf_messages::XLanelet lanelet);

    /**
     * Compute new speed to be able to stop in X meters
     * @param wishedSpeed the originally desired speed (our max speed)
     * @param stoppingDistance the distance (=X meters)
     * @return the speed to be able to stop in x meters or the original speed
     * if the computed value would exceed the wished speed
     */
    double computeSpeedToStopInXMeters(double wishedSpeed, double stoppingDistance);
};