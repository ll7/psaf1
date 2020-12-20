// #include "psaf_steering/plugin_local_planner.h"
#include "psaf_local_planner/plugin_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>


//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(psaf_local_planner::PsafLocalPlanner, nav_core::BaseLocalPlanner)

namespace psaf_local_planner {
    PsafLocalPlanner::PsafLocalPlanner() : odom_helper("/carla/ego_vehicle/odometry") {
        std::cout << "Hi";
    }

    void velocityCallback(const geometry_msgs::Twist& msg) {

    }

    /**
     * Constructs the local planner.
     */ 
    void PsafLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS *costmap_ros) {
        ros::NodeHandle private_nh("~/" + name);
        g_plan_pub = private_nh.advertise<nav_msgs::Path>("psaf_global_plan", 1);
        l_plan_pub = private_nh.advertise<nav_msgs::Path>("psaf_local_plan", 1);
        vel_sub = private_nh.subscribe("psaf_velocity_plan", 10, velocityCallback);
        planner_util.initialize(tf, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }
    

    PsafLocalPlanner::~PsafLocalPlanner() {
        g_plan_pub.shutdown();
        vel_sub.shutdown();
    }
        

    /**
     * Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
     */
    bool PsafLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
        return true;
    }
    

    /**
     * Check if the goal pose has been achieved by the local planner.
     */
    bool PsafLocalPlanner::isGoalReached() {
        return false;
    }
    
    /**
     * Set the plan that the local planner is following.
     */
    bool PsafLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
        planner_util.setPlan(plan);
        return true;
    }

    void PsafLocalPlanner::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, l_plan_pub);
    }

    void PsafLocalPlanner::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, g_plan_pub);
    }


    /**
     * Compute relative angle and distance between a target_location and a current_location
     */
    void compute_magnitude_angle(geometry_msgs::Pose target_location, geometry_msgs::Pose current_location) {
        
        tf2::Transform target_transform;
        tf2::Transform current_transform;
        tf2::convert(target_location, target_transform);
        tf2::convert(current_location, current_transform);
        
        tf2::Quaternion quat;
        tf2::convert(current_location.orientation, quat);
        
        // unsure whether this is the same value as in python
        auto orientation = tf2::Quaternion(current_location.orientation.x, current_location.orientation.y, current_location.orientation.z, current_location.orientation.w).getAngle();
        
        /*
        // angle of vehicle
        q = (current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w);
        _, _, yaw = euler_from_quaternion(q);
        orientation = math.degrees(yaw);
        */

        // vector from vehicle to target point and distance
        tf2::Vector3 target_vector = tf2::Vector3(target_location.position.x - current_location.position.x, target_location.position.y - current_location.position.y, 0);
        target_vector.length();
        auto dist_target = target_vector;
        /*target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y]);
        dist_target = np.linalg.norm(target_vector);

        // vector of the car and absolut angle between vehicle and target point
        // angle = (a o b) / (|a|*|b|), here: |b| = 1
        forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))]);
        c = np.clip(np.dot(forward_vector, target_vector) / dist_target, -1.0, 1.0);
        d_angle = math.degrees(math.acos(c));

        //make angle negative or positive
        cross = np.cross(forward_vector, target_vector);
        if (cross < 0) {
            d_angle *= -1.0;
        }

        return dist_target, d_angle
        */
    }
}