#include "psaf_local_planner/plugin_local_planner.h"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(psaf_local_planner::PsafLocalPlanner, nav_core::BaseLocalPlanner)

namespace psaf_local_planner
{
    PsafLocalPlanner::PsafLocalPlanner() : odom_helper("/carla/ego_vehicle/odometry"), global_plan({}),
                                            initialized(false), closest_point_local_plan(2),
                                            lookahead_factor(3.5), target_velocity(15), min_velocity(5),
                                            goal_reached(false), estimate_curvature_distance(50), check_collision_max_distance(40),
                                            slow_car_ahead_counter(0), slow_car_ahead_published(false), obstacle_msg_id_counter(0), duration_factor(2.0)
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
     * 
     * Overrides the nav_core::BaseLocalPlanner method; Called upon node creation
     * @param name: Name of the node in which this plugin runs
     * @param tf: ?
     * @param costmap_ros: Pointer to the costmap internal to move_base
     */
    void PsafLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized)
        {
            ros::NodeHandle private_nh("~/" + name);
            g_plan_pub = private_nh.advertise<nav_msgs::Path>("psaf_global_plan", 1);
            obstacle_pub = private_nh.advertise<psaf_messages::Obstacle>("/psaf/planning/obstacle", 1);
            debug_pub = private_nh.advertise<visualization_msgs::MarkerArray>("debug", 1);

            global_plan_extended_sub = private_nh.subscribe("/psaf/xroute", 10, &PsafLocalPlanner::globalPlanExtendedCallback, this);
            planner_util.initialize(tf, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
            this->costmap_ros = costmap_ros;

            dyn_serv = new dynamic_reconfigure::Server<PsafLocalPlannerParameterConfig>(private_nh);
            dynamic_reconfigure::Server<PsafLocalPlannerParameterConfig>::CallbackType f = boost::bind(&PsafLocalPlanner::reconfigureCallback, this, _1, _2);
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

            if (global_plan.size() <= 1)
            {
                ROS_INFO("Goal reached");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                goal_reached = true;
            }
            else
            {
                estimateCurvatureAndSetTargetVelocity(current_pose.pose);
                auto target_point = findLookaheadTarget();

                double angle = computeSteeringAngle(target_point.pose, current_pose.pose);
                double distance, relX, relY;

                double velocity_distance_diff;

                if (target_velocity > 0 && !checkDistanceForward(distance, relX, relY)) {
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

                    ROS_INFO("distance forward: %f, max velocity: %f", distance, target_velocity);
                }

                checkForSlowCar(velocity_distance_diff);

                cmd_vel.linear.x = target_velocity - velocity_distance_diff;
                cmd_vel.angular.z = angle;
            }
        }
        else
        {
            ROS_WARN("Called compute velocity before being inited");
        }

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
        if (to_delete > 0) {
            global_plan.erase(global_plan.begin(), global_plan.begin() + to_delete);
            
            auto route_iter = global_route.begin();
            while (route_iter != global_route.end() && to_delete > 0) {
                auto &lanelet = *route_iter;
                int size = lanelet.route_portion.size();

                if (to_delete >= size) {
                    route_iter = global_route.erase(route_iter);
                    to_delete -= size;
                } else {
                    lanelet.route_portion.erase(lanelet.route_portion.begin(), lanelet.route_portion.begin() + to_delete);
                    to_delete = 0;
                }
            }
        }

        // validate whether the plans are the same length; TODO: Remove when sufficiently validated 
        int route_len = 0;
        for (auto lanelet : global_route) {
            route_len += lanelet.route_portion.size();
        }
        
        assert(route_len == global_plan.size());
    }

    void PsafLocalPlanner::globalPlanExtendedCallback(const psaf_messages::XRoute &msg)
    {
        ROS_INFO("RECEIVED MESSAGE: %d", msg.id);
        bool traffic_rules = true;
        ros::param::get("/path_provider/respect_traffic_rules",traffic_rules);
        // validate duration of routes only if a global route already exists
        if (global_route.size() > 0 && !goal_reached && traffic_rules) {
            std::vector<psaf_messages::XLanelet> new_global_route = {};
            new_global_route = msg.route;
            float new_duration = 0;
            float old_duration = 0;
            // calculate duration of new route
            for (auto lanelet : new_global_route) {
                new_duration += lanelet.route_portion[lanelet.route_portion.size()-1].duration;
            }
            // duration of first lanelet of current route
            old_duration += global_route[0].route_portion[global_route[0].route_portion.size()-1].duration;
            // subtract already covered route fragment from duration of whole lanelet
            old_duration -= global_route[0].route_portion[0].duration;
            // calculate remaining duration of current route
            for (int i = 1; i < global_route.size(); i++ ) {
                old_duration += global_route[i].route_portion[global_route[i].route_portion.size()-1].duration;
            }
            // if new route is too slow, keep old one
            if (new_duration > (duration_factor * old_duration)){
                ROS_INFO("New Route is at least %f x slower than Current Route -> Current Route is kept", duration_factor);
                return;
            }

        }
        // validate length of routes only if a global route already exists, no traffic rules case
        if (global_route.size() > 0 && !goal_reached && !traffic_rules) {
            std::vector<psaf_messages::XLanelet> new_global_route = {};
            new_global_route = msg.route;
            float new_distance = 0;
            float old_distance = 0;
            // calculate distance of new route
            for (auto lanelet : new_global_route) {
                new_distance += lanelet.route_portion[lanelet.route_portion.size()-1].duration;
            }
            // distance of first lanelet of current route
            old_distance += global_route[0].route_portion[global_route[0].route_portion.size()-1].duration;
            // subtract already covered route fragment from distance of whole lanelet
            old_distance -= global_route[0].route_portion[0].duration;
            // calculate remaining distance of current route
            for (int i = 1; i < global_route.size(); i++ ) {
                old_distance += global_route[i].route_portion[global_route[i].route_portion.size()-1].duration;
            }
            // if new route is too long, keep old one
            if (new_distance > (duration_factor * old_distance)){
                ROS_INFO("New Route is at least %f x slower than Current Route -> Current Route is kept", duration_factor);
                return;
            }

        }


        global_route = msg.route;
        goal_reached = false;

        std::vector<geometry_msgs::PoseStamped> points = {};

        unsigned int counter = 0;
        for (auto lanelet : global_route) {
            for (auto point : lanelet.route_portion) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.seq = counter++;
                pose.pose.orientation.w = 1;
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = point.z;

                points.push_back(pose);
            }
        }

        global_plan = points;
        planner_util.setPlan(global_plan);
        publishGlobalPlan(global_plan);
    }

}

