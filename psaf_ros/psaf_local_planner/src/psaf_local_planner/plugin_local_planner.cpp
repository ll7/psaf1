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
                                           bufferSize(1000), initialized(false), closest_point_local_plan(3),
                                           lookahead_factor(3), max_velocity(15), target_velocity(15), min_velocity(5),
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
                // ROS_INFO("Nearest waypoint to <%f, %f> is <%f, %f>\n", current_pose.pose.position.x, current_pose.pose.position.y, w.pose.position.x, w.pose.position.y);
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

            deleteOldPoints();
            fillPointBuffer();
            publishLocalPlan(local_plan);

            if (local_plan.empty() && global_plan.empty())
            {
                ROS_INFO("Goal reached");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                goal_reached = true;
            }
            else
            {
                estimate_curvature_and_set_target_velocity(current_pose.pose);
                // int index = (target_velocity / lookahead_factor) + 4;
                // auto target_point = &local_plan[index];

                auto target_point = find_lookahead_target();

                double angle = compute_steering_angle(target_point.pose, current_pose.pose);
                double distance;
                
                if (target_velocity > 0 && !check_distance_forward(distance)) {
                    target_velocity *= boost::algorithm::clamp((distance - 5) / (pow(max_velocity * 0.36, 2)), 0, 1);
                    ROS_INFO("distance forward: %f, max distance: %f", distance, pow(max_velocity * 0.36, 2));
                }

                cmd_vel.linear.x = target_velocity;
                cmd_vel.angular.z = angle;
            }
        }
        else
        {
            ROS_WARN("Called compute velocity before being inited");
        }

        return true;
    }

    geometry_msgs::PoseStamped& PsafLocalPlanner::find_lookahead_target() {
        tf2::Vector3 last_point, current_point, acutal_point;
        tf2::convert(current_pose.pose.position, last_point);
        acutal_point = last_point;

        double desired_distance = (target_velocity / lookahead_factor) + 3;
        double sum_distance = 0;

        for (auto it = local_plan.begin(); it != local_plan.end(); ++it)
        {
            geometry_msgs::PoseStamped &w = *it;
            tf2::convert(w.pose.position, current_point);
            sum_distance += tf2::tf2Distance(last_point, current_point);

            if (sum_distance > desired_distance) {
                return w;
            }

            last_point = current_point;
        }

        auto &last_stamp = *local_plan.end();
        return last_stamp;
    }


    /**
     * Check if the goal pose has been achieved by the local planner.
     */
    bool PsafLocalPlanner::isGoalReached()
    {
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
    double PsafLocalPlanner::compute_steering_angle(geometry_msgs::Pose target_location, geometry_msgs::Pose current_location)
    {
        tf2::Transform target_transform;
        tf2::Transform current_transform;
        tf2::convert(target_location, target_transform);
        tf2::convert(current_location, current_transform);

        // Rotation around z axis of the car
        auto orientation = tf2::getYaw(current_transform.getRotation());

        // vector from vehicle to target point and distance
        tf2::Vector3 target_vector = (target_transform.getOrigin() - current_transform.getOrigin()).normalize();

        // vector of the car and absolut angle between vehicle and target point
        // angle = (a o b) / (|a|*|b|), here: |b| = 1
        // using atan instead of acos here to get the correct sign with the direction of the angle
        tf2::Vector3 forward_vector = tf2::Vector3(cos(orientation), sin(orientation), 0);
        double unclamped_angle = atan2(
            forward_vector.getX() * target_vector.getY() - forward_vector.getY() * target_vector.getX(),
            forward_vector.getX() * target_vector.getX() + forward_vector.getY() * target_vector.getY()
        );

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

        return d_angle;
    }

    void PsafLocalPlanner::estimate_curvature_and_set_target_velocity(geometry_msgs::Pose current_location)
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
        auto fact = boost::algorithm::clamp(sum_angle * 10 / sum_distance, 0, 1);
        target_velocity = (max_velocity - fact * (max_velocity - min_velocity));
    }

    /**
     * Returns false if it failed because something is close; true if out of bounds
     */
    bool PsafLocalPlanner::check_distance_forward(double& distance)
    {
        tf2::Vector3 last_point, current_point, acutal_point;
        tf2::convert(current_pose.pose.position, last_point);
        acutal_point = last_point;

        double sum_distance = 0;
        int count_error = 0;

        for (auto it = local_plan.begin(); it != local_plan.end(); ++it)
        {
            const geometry_msgs::PoseStamped &w = *it;
            tf2::convert(w.pose.position, current_point);
            sum_distance += tf2::tf2Distance(last_point, current_point);

            unsigned int cx, cy;
            if (costmap_ros->getCostmap()->worldToMap(current_point.getX(), current_point.getY(), cx, cy))
            {
                unsigned char cost = costmap_ros->getCostmap()->getCost(cx, cy);

                if (cost > 128 && cost != costmap_2d::NO_INFORMATION)
                {
                    count_error += 1;
                    if (count_error >= 2)
                    {
                        ROS_WARN("cost is %i at %f %f", cost, current_point.getX() - acutal_point.getX(), current_point.getY() - acutal_point.getY());                
                        distance = sum_distance;
                        return false;
                    }
                }
                else
                {
                    count_error = 0;
                }
            }

            last_point = current_point;
        }

        

        distance = sum_distance;
        return true;
    }
} // namespace psaf_local_planner