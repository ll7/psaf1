// #include "psaf_steering/plugin_local_planner.h"
#include "psaf_local_planner/plugin_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <boost/algorithm/clamp.hpp>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(psaf_local_planner::PsafLocalPlanner, nav_core::BaseLocalPlanner)

namespace psaf_local_planner
{
    PsafLocalPlanner::PsafLocalPlanner() : odom_helper("/carla/ego_vehicle/odometry"), local_plan({}),
                                           bufferSize(100), initialized(false), closest_point_local_plan(3),
                                           lookahead_factor(1), max_velocity(15), target_velocity(15), min_velocity(5),
                                           goal_reached(false)
    {
        std::cout << "Hi";
    }

    /**
     * Deletes the points in the local plan that have been driven over
     */
    void PsafLocalPlanner::deleteOldPoints()
    {
        std::vector<geometry_msgs::PoseStamped>::iterator it = local_plan.begin();

        // std::string s = (std::string)"x: " + std::to_string(current_pose.pose.position.x) + "z: " + std::to_string(current_pose.pose.position.z) + "z: " + std::to_string(current_pose.pose.position.z);

        while (it != local_plan.end())
        {
            const geometry_msgs::PoseStamped &w = *it;
            // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
            double x_diff = current_pose.pose.position.x - w.pose.position.x;
            double y_diff = current_pose.pose.position.y - w.pose.position.y;
            double distance_sq = x_diff * x_diff + y_diff * y_diff;
            if (distance_sq > closest_point_local_plan * closest_point_local_plan)
            {
                ROS_INFO("Nearest waypoint to <%f, %f> is <%f, %f>\n", current_pose.pose.position.x, current_pose.pose.position.y, w.pose.position.x, w.pose.position.y);
                break;
            }
            it = local_plan.erase(it);
        }
    }

    /**
     * Fills the local path with the next 100 points of the path and removes them from the global plan
     */
    void PsafLocalPlanner::fillPointBuffer()
    {
        if (global_plan.empty())
        {
            return;
        }
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();

        //> oder >= size?
        while (bufferSize > local_plan.size() && it != global_plan.end())
        {
            const geometry_msgs::PoseStamped &w = *it;
            local_plan.push_back(w);

            it = global_plan.erase(it);
        }
    }

    void velocityCallback(const geometry_msgs::Twist &msg)
    {
    }

    /**
     * Constructs the local planner.
     */
    void PsafLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized)
        {
            ros::NodeHandle private_nh("~/" + name);
            g_plan_pub = private_nh.advertise<nav_msgs::Path>("psaf_global_plan", 1);
            l_plan_pub = private_nh.advertise<nav_msgs::Path>("psaf_local_plan", 1);
            debug_pub = private_nh.advertise<visualization_msgs::MarkerArray>("debug", 1);

            vel_sub = private_nh.subscribe("psaf_velocity_plan", 10, velocityCallback);
            planner_util.initialize(tf, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
            this->costmap_ros = costmap_ros;

            initialized = true;
        }
        else
        {
            ROS_WARN("Node is already inited");
        }
    }

    PsafLocalPlanner::~PsafLocalPlanner()
    {
        g_plan_pub.shutdown();
        vel_sub.shutdown();
    }

    /**
     * Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
     */
    bool PsafLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (initialized)
        {
            costmap_ros->getRobotPose(current_pose);
            ROS_INFO("x: %f, y: %f, z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

            deleteOldPoints();
            fillPointBuffer();
            publishLocalPlan(local_plan);

            ROS_INFO("Computed Velocity, len of plan: %ld", local_plan.size());
            if (local_plan.empty() && global_plan.empty())
            {
                ROS_INFO("Goal reached");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                goal_reached = true;
            }
            else
            {
                int index = (target_velocity / lookahead_factor);
                auto target_point = &local_plan[index];

                float mag;
                float angle;
                compute_magnitude_angle(target_point->pose, current_pose.pose, mag, angle);
                estimate_curvature(current_pose.pose);

                cmd_vel.linear.x = target_velocity;
                cmd_vel.angular.z = angle;
            }
        }
        else
        {
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
    bool PsafLocalPlanner::isGoalReached()
    {
        ROS_INFO("called: %i", goal_reached);
        return goal_reached && local_plan.empty() && global_plan.empty();
    }

    /**
     * Set the plan that the local planner is following.
     */
    bool PsafLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        planner_util.setPlan(plan);
        publishGlobalPlan(plan);
        global_plan = plan;
        goal_reached = false;
        return true;
    }

    void PsafLocalPlanner::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, l_plan_pub);
    }

    void PsafLocalPlanner::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, g_plan_pub);
    }

    /**
     * Compute relative angle and distance between a target_location and a current_location
     */
    void PsafLocalPlanner::compute_magnitude_angle(geometry_msgs::Pose target_location, geometry_msgs::Pose current_location, float &magnitude, float &angle)
    {
        tf2::Transform target_transform;
        tf2::Transform current_transform;
        tf2::convert(target_location, target_transform);
        tf2::convert(current_location, current_transform);

        // Rotation around z axis of the car
        auto orientation = tf2::getYaw(current_transform.getRotation());

        // vector from vehicle to target point and distance
        tf2::Vector3 target_vector = target_transform.getOrigin() - current_transform.getOrigin();
        auto dist_target = target_vector.length();
        target_vector.normalize();

        // vector of the car and absolut angle between vehicle and target point
        // angle = (a o b) / (|a|*|b|), here: |b| = 1
        // using atan instead of acos here to get the correct sign with the direction of the angle
        tf2::Vector3 forward_vector = tf2::Vector3(cos(orientation), sin(orientation), 0);
        double unclamped_angle = atan2(
            forward_vector.getX() * target_vector.getY() - forward_vector.getY() * target_vector.getX(),
            forward_vector.getX() * target_vector.getX() + forward_vector.getY() * target_vector.getY());
        ROS_INFO("Angle: %f", unclamped_angle);

        // Limit angle to the limits of the car
        auto d_angle = boost::algorithm::clamp(unclamped_angle, -1.2, 1.2);

        // Debug data for the direction and the angle between the car and the road
        auto marker1 = visualization_msgs::Marker();
        auto marker2 = visualization_msgs::Marker();
        auto markers = visualization_msgs::MarkerArray();

        marker1.type = visualization_msgs::Marker::ARROW;
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.ns = "current";
        marker1.header.frame_id = "map";
        marker1.header.stamp = ros::Time::now();
        marker1.color.a = 1.0;
        marker1.color.r = 1.0;
        marker1.pose = current_location;
        marker1.scale.x = 1.0;
        marker1.scale.y = 0.1;
        marker1.scale.z = 0.1;

        marker2.type = visualization_msgs::Marker::ARROW;
        marker2.action = visualization_msgs::Marker::ADD;
        marker2.ns = "target";
        marker2.header.frame_id = "ego_vehicle";
        marker2.header.stamp = ros::Time::now();
        marker2.color.a = 1.0;
        marker2.color.r = 0.0;
        marker2.color.g = 1.0;
        marker2.pose.position.x = 0;
        marker2.pose.position.y = 0;
        marker2.pose.position.z = 0;
        marker2.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), unclamped_angle));
        marker2.scale.x = 1.0;
        marker2.scale.y = 0.1;
        marker2.scale.z = 0.1;

        markers.markers = {marker1, marker2};
        debug_pub.publish(markers);

        magnitude = dist_target;
        angle = d_angle;
    }

    void PsafLocalPlanner::estimate_curvature(geometry_msgs::Pose current_location)
    {
        tf2::Vector3 point1, point2, point3;
        auto it = local_plan.begin();

        tf2::convert(current_location.position, point1);
        const geometry_msgs::PoseStamped &w = *it;
        tf2::convert(w.pose.position, point2);

        ++it;
        double sum_distance = tf2::tf2Distance2(point1, point2);
        double sum_angle = 0;

        for (; it != local_plan.end(); ++it)
        {
            const geometry_msgs::PoseStamped &w = *it;
            tf2::convert(w.pose.position, point3);

            sum_distance += tf2::tf2Distance(point2, point3);
            auto v1 = (point2 - point1);
            auto v2 = (point3 - point2);

            double angle = v1.angle(v2);
            if (isfinite(angle))
                sum_angle += abs(angle);

            point1 = point2;
            point2 = point3;
        }

        ROS_INFO("curvature: %f; distance %f", sum_angle, sum_distance);
        auto fact = boost::algorithm::clamp(sum_angle * 20 / sum_distance, 0, 1);
        target_velocity = (max_velocity - fact * (max_velocity - min_velocity));

    }
} // namespace psaf_local_planner