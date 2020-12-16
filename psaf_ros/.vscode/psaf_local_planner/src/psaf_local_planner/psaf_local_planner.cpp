// #include "psaf_steering/plugin_local_planner.h"
#include "psaf_local_planner/plugin_local_planner.h"




namespace psaf_local_planner {
    PsafLocalPlanner::PsafLocalPlanner() {
        std::cout << "Hi";
    }

    void velocityCallback(const geometry_msgs::Twist& msg) {

    }

    /**
     * Constructs the local planner.
     */ 
    void PsafLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS *costmap_ros) {
        ros::NodeHandle private_nh("~/" + name);
        g_plan_pub = private_nh.advertise<nav_msgs::Path>("psaf_global_plan", 1, true);
        vel_sub = private_nh.subscribe("psaf_velocity_plan", 10, velocityCallback);
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
        return true;
    }
    
    /**
     * Set the plan that the local planner is following.
     */
    bool PsafLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
        return true;
    }
}