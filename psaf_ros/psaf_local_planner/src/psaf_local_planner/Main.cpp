#include "psaf_local_planner/plugin_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <boost/algorithm/clamp.hpp>

PLUGINLIB_EXPORT_CLASS(psaf_local_planner::PsafLocalPlanner, nav_core::BaseLocalPlanner)

namespace psaf_local_planner
{
    PsafLocalPlanner::PsafLocalPlanner() : odom_helper("/carla/ego_vehicle/odometry"), global_plan({}),
                                           bufferSize(1000), initialized(false), closest_point_local_plan(2),
                                           lookahead_factor(3.5), max_velocity(15), target_velocity(15), min_velocity(5),
                                           goal_reached(false), estimate_curvature_distance(50), check_collision_max_distance(40),
                                           slow_car_ahead_counter(0), slow_car_ahead_published(false), obstacle_msg_id_counter(0)
    {
        std::cout << "Hi";
    }

    PsafLocalPlanner::~PsafLocalPlanner()
    {
        g_plan_pub.shutdown();
        vel_sub.shutdown();
        delete dyn_serv;
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
            obstacle_pub = private_nh.advertise<psaf_messages::Obstacle>("/psaf/planning/obstacle", 1);
            //l_plan_pub = private_nh.advertise<nav_msgs::Path>("psaf_local_plan", 1);
            debug_pub = private_nh.advertise<visualization_msgs::MarkerArray>("debug", 1);

            global_plan_extended_sub = private_nh.subscribe("psaf_global_plan_extended_TODODODODODODODO", 10, &PsafLocalPlanner::globalPlanExtendedCallback, this);
            planner_util.initialize(tf, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
            this->costmap_ros = costmap_ros;

            dyn_serv = new dynamic_reconfigure::Server<PsafLocalPlannerParameterConfig>(private_nh);
            dynamic_reconfigure::Server<PsafLocalPlannerParameterConfig>::CallbackType f = boost::bind(&PsafLocalPlanner::reconfigure_callback, this, _1, _2);
            dyn_serv->setCallback(f);

            initialized = true;
        }
        else
        {
            ROS_WARN("Node is already inited");
        }
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
            // fillPointBuffer();
            // publishLocalPlan(local_plan);

            if (global_plan.size() <= 1)
            {
                ROS_INFO("Goal reached");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                goal_reached = true;
            }
            else
            {
                estimate_curvature_and_set_target_velocity(current_pose.pose);
                auto target_point = find_lookahead_target();

                double angle = compute_steering_angle(target_point.pose, current_pose.pose);
                double distance, relX, relY;

                double velocity_distance_diff;

                if (target_velocity > 0 && !check_distance_forward(distance, relX, relY)) {
                    if (distance < 5) {
                        ROS_INFO("attempting to stop");
                        velocity_distance_diff = target_velocity;
                    } else {
                        // TODO: validate if working
                        // slow formula, working okay ish
                        velocity_distance_diff = target_velocity - std::min(target_velocity, 25.0/18.0 * (-1 + std::sqrt(1 + 4 * (distance - 5))));
                        // faster formula, requires faster controller
                        //velocity_distance_diff = target_velocity - std::min(target_velocity, 25.0/9.0 * std::sqrt(distance - 5));
                    }

                    // target_velocity *= boost::algorithm::clamp((distance - 5) / (pow(max_velocity * 0.36, 2)), 0, 1);
                    ROS_INFO("distance forward: %f, max velocity: %f", distance, target_velocity);
                }

                checkForSlowCar(velocity_distance_diff);

                //std::vector<RaytraceCollisionData> collisions = {};
                //raytraceSemiCircle(M_PI * 3/2, 30, collisions);


                cmd_vel.linear.x = target_velocity - velocity_distance_diff;
                cmd_vel.angular.z = angle;
            }
        }
        else
        {
            ROS_WARN("Called compute velocity before being inited");
        }

        /*auto x = costmap_ros->getCostmap()->getSizeInCellsX();
        auto y = costmap_ros->getCostmap()->getSizeInCellsY();
        costmap_ros->getCostmap()->resetMaps();*/

        return true;
    }

    /**
     * Deletes the points in the local plan that have been driven over
     */
    void PsafLocalPlanner::deleteOldPoints()
    {
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        double closest_point_local_plan_sq = closest_point_local_plan* closest_point_local_plan;

        double last_dist = 1e100;
        std::vector<geometry_msgs::PoseStamped>::iterator last_it = it;
        std::vector<geometry_msgs::PoseStamped>::iterator closest_it = it;

        int to_delete = -1;

        while (it != global_plan.end())
        {
            const geometry_msgs::PoseStamped &w = *it;

            double x_diff = current_pose.pose.position.x - w.pose.position.x;
            double y_diff = current_pose.pose.position.y - w.pose.position.y;
            double distance_sq = x_diff * x_diff + y_diff * y_diff;

            // find the closest point
            if (last_dist > distance_sq) {
                closest_it = it;
                last_dist  = distance_sq;
                to_delete++;
            } else {
                // ensure that the closest point is ahead of the car when it is inside the bounds of the car
                if (distance_sq < closest_point_local_plan_sq) {
                    closest_it = it;
                    to_delete++;
                } else {
                    break;
                }
            }

            it++;
        }

        // actually delete the points now
        if (to_delete > 0)
            global_plan.erase(global_plan.begin(), global_plan.begin() + to_delete);

    }

    void PsafLocalPlanner::globalPlanExtendedCallback(const geometry_msgs::Twist &msg)
    {
    }

}

