#pragma once

#include <base_local_planner/goal_functions.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <base_local_planner/line_iterator.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace psaf_local_planner {
    class RaytraceCollisionData {
        public:
            RaytraceCollisionData(double x, double y, double angle, double distance);
            double x, y, angle, distance;
    };

    class CostmapRaytracer {
        public:
            CostmapRaytracer();
            CostmapRaytracer(costmap_2d::Costmap2DROS *costmap_ros, geometry_msgs::PoseStamped *current_pose, ros::Publisher *debug_pub);

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
             * Checks for no movement; has to be called multiple times to be expressive
             * 
             * @param angle: the angle in rad around the car; circle is open to the back of the car if it isn't a full circle
             * @param distance: max distance that should be looked at by the raytrace
             * @return returns true if there was no movement between this and the last function call
             */
            bool checkForNoMovement(double angle, double distance);

            /**
             * Epsilon that is used to determine whether two points are the same collision on the costmap
             * 
             * calulcation uses manhattan norm for the distance calucaltion
             */
            static constexpr double MANHATTAN_EPSILON = 0.4;
        private:
            /** The pointer to the costmap, updates automatically when the costmap changes */
            costmap_2d::Costmap2DROS *costmap_ros;


            /** pointer to the current pose of the vehicle; targeted field should be updated */
            geometry_msgs::PoseStamped *current_pose;

            /** Publisher for debug messages; e.g. arrows along path, obstacle bobbels */
            ros::Publisher *debug_pub;

            std::vector<RaytraceCollisionData> second_last_raytrace_results;
            std::vector<RaytraceCollisionData> last_raytrace_results;
    };
};