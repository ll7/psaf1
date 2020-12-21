// #include "psaf_steering/plugin_local_planner.h"
#include "psaf_local_planner/plugin_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <boost/algorithm/clamp.hpp>


//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(psaf_local_planner::PsafLocalPlanner, nav_core::BaseLocalPlanner)

namespace psaf_local_planner {
    PsafLocalPlanner::PsafLocalPlanner() : odom_helper("/carla/ego_vehicle/odometry"),  local_plan({}), 
                                            bufferSize(100), initialized(false) {
        std::cout << "Hi";
    }

    /**
     * Deletes the points in the local plan that have been driven over
     */
    void PsafLocalPlanner::deleteOldPoints() {
        std::vector<geometry_msgs::PoseStamped>::iterator it = local_plan.begin();

        // std::string s = (std::string)"x: " + std::to_string(current_pose.pose.position.x) + "z: " + std::to_string(current_pose.pose.position.z) + "z: " + std::to_string(current_pose.pose.position.z);
        
        while(it != local_plan.end()){
            const geometry_msgs::PoseStamped& w = *it;
            // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
            double x_diff = current_pose.pose.position.x - w.pose.position.x;
            double y_diff = current_pose.pose.position.y - w.pose.position.y;
            double distance_sq = x_diff * x_diff + y_diff * y_diff;
            if(distance_sq < 1){
                ROS_INFO("Nearest waypoint to <%f, %f> is <%f, %f>\n", current_pose.pose.position.x, current_pose.pose.position.y, w.pose.position.x, w.pose.position.y);
                break;
            }
            it = local_plan.erase(it);
        }
    }

    /**
     * Fills the local path with the next 100 points of the path and removes them from the global plan
     */
    void PsafLocalPlanner::fillPointBuffer() {
        if (global_plan.empty()) {
            return;
        }
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        
        //> oder >= size?
        while (bufferSize > local_plan.size() && it != global_plan.end()) {
            const geometry_msgs::PoseStamped& w = *it;
            local_plan.push_back(w);

            it = global_plan.erase(it);
        }
    }

    void velocityCallback(const geometry_msgs::Twist& msg) {
        
    }

    /**
     * Constructs the local planner.
     */ 
    void PsafLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS *costmap_ros) {
        if (!initialized) {
            ros::NodeHandle private_nh("~/" + name);
            g_plan_pub = private_nh.advertise<nav_msgs::Path>("psaf_global_plan", 1);
            l_plan_pub = private_nh.advertise<nav_msgs::Path>("psaf_local_plan", 1);
            vel_sub = private_nh.subscribe("psaf_velocity_plan", 10, velocityCallback);
            planner_util.initialize(tf, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
            this->costmap_ros = costmap_ros;

            initialized = true;
        } else {
            ROS_WARN("Node is already inited");
        }
    }
    

    PsafLocalPlanner::~PsafLocalPlanner() {
        g_plan_pub.shutdown();
        vel_sub.shutdown();
    }
        

    /**
     * Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
     */
    bool PsafLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
        if (initialized) {
            costmap_ros->getRobotPose(current_pose);
            ROS_INFO("x: %f, y: %f, z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

            deleteOldPoints();
            fillPointBuffer();
            publishLocalPlan(local_plan);
            
            ROS_INFO("Computed Velocity, len of plan: %ld", local_plan.size());
            if (local_plan.empty() && global_plan.empty()) {
                ROS_INFO("Goal reached");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            } else {
                auto target_point = &local_plan[0];


                tf2::Quaternion target_quat;
                tf2::Quaternion current_quat;
                tf2::convert(target_point->pose.orientation, target_quat);
                tf2::convert(current_pose.pose.orientation, current_quat);

            
                
                float mag;
                float angle;
                compute_magnitude_angle(current_pose.pose, target_point->pose, mag, angle);

                // returns the **half** angle between the two vectors
                auto angle_tf2 = tf2::angle(current_quat, target_quat);
                // clamp to 60 degrees
                angle_tf2 = boost::algorithm::clamp(angle_tf2 * 2, -1.0472, 1.0472);

                ROS_INFO("angle tf2: %f; own: %f", angle_tf2, angle );

                cmd_vel.linear.x = 5;
                cmd_vel.angular.z = boost::algorithm::clamp(angle, -1.0472, 1.0472);
            }


        } else {
            ROS_WARN("Called compute velocity before being inited");
        }
        
        /*self.del_old_points()

        if len(self.local_plan) < self.buffer_size:
            self.fill_point_buffer()

        if not self.local_plan and not self.global_plan:
            self.target_reached = True
            rospy.loginfo("Target reached!")
            self.control_cmd.steering_angle = 0.0
            self.control_cmd.speed = 0.0
        else:
            target_point = self.local_plan[0]
            _, angle = self.compute_magnitude_angle(target_point, self.current_location, self.current_orientation)
            self.control_cmd.steering_angle = math.radians(np.clip(angle, -60.0, 60.0))
            # self.control_cmd.speed = (self.target_speed - abs(self.control_cmd.steering_angle) * (
            #         self.target_speed - self.min_speed)) / 3.6

            d, a = self.estimate_curvature(self.current_location, self.current_orientation, self.local_plan)
            rospy.loginfo('curvature: ' + str(a))
            fact = np.clip(a/100, 0, 1)
            self.control_cmd.speed = (self.target_speed - fact * (self.target_speed - self.min_speed)) / 3.6
        */
        
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
        publishGlobalPlan(plan);
        global_plan = plan;
        return true;
    }

    void PsafLocalPlanner::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, l_plan_pub);
    }

    void PsafLocalPlanner::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, g_plan_pub);
    }


    /**
     * Compute relative angle and distance between a target_location and a current_location
     */
    void PsafLocalPlanner::compute_magnitude_angle(geometry_msgs::Pose target_location, geometry_msgs::Pose current_location, float &magnitude, float &angle) {
        tf2::Transform target_transform;
        tf2::Transform current_transform;
        tf2::convert(target_location, target_transform);
        tf2::convert(current_location, current_transform);
        
        tf2::Quaternion quat;
        tf2::convert(current_location.orientation, quat);
        
        // unsure whether this is the same value as in python
        auto orientation = current_transform.getRotation().getAxis().getZ();
        
        /*
        // angle of vehicle
        q = (current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w);
        _, _, yaw = euler_from_quaternion(q);
        orientation = math.degrees(yaw);
        */

        // vector from vehicle to target point and distance
        tf2::Vector3 target_vector = tf2::Vector3(target_location.position.x - current_location.position.x, target_location.position.y - current_location.position.y, 0);
        target_vector.length();
        auto dist_target = target_vector.length();
        target_vector.normalize();
        /*target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y]);
        dist_target = np.linalg.norm(target_vector);
        */
        
        // vector of the car and absolut angle between vehicle and target point
        // angle = (a o b) / (|a|*|b|), here: |b| = 1
        tf2::Vector3 forward_vector = tf2::Vector3(cos(orientation), sin(orientation), 0);
        auto c = boost::algorithm::clamp(forward_vector.dot(target_vector) / dist_target, -1.0, 1.0);
        auto d_angle = acos(c);

        /*
        forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))]);
        c = np.clip(np.dot(forward_vector, target_vector) / dist_target, -1.0, 1.0);
        d_angle = math.degrees(math.acos(c));

        */

        //make angle negative or positive
        
        auto cross = forward_vector.cross(target_vector);
        if (cross < 0) {
            d_angle *= -1.0;
        }
        /*cross = np.cross(forward_vector, target_vector);
        if (cross < 0) {
            d_angle *= -1.0;
        }

        return dist_target, d_angle
        */


        magnitude = dist_target;
        angle = d_angle;
    }
}