#include "psaf_local_planner/plugin_local_planner.h"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(psaf_local_planner::PsafLocalPlanner, nav_core::BaseLocalPlanner)

namespace psaf_local_planner
{
    PsafLocalPlanner::PsafLocalPlanner() : odom_helper("/carla/ego_vehicle/odometry"), global_plan({}),
                                            initialized(false), closest_point_local_plan(2),
                                            lookahead_factor(3.5), target_velocity(15), min_velocity(5),
                                            goal_reached(false), estimate_curvature_distance(50), check_collision_max_distance(40),
                                            slow_car_ahead_counter(0), slow_car_ahead_published(false), obstacle_msg_id_counter(0), 
                                            duration_factor(2.0), distance_factor(2.0), respect_traffic_rules(true), max_points_smoothing(10), 
                                            lane_change_direction(0), lane_change_direction_calculated(false), lookahead_factor_const_additive(1)
    {
        std::cout << "Hi";
        this->state_machine = new LocalPlannerStateMachine();
    }

    PsafLocalPlanner::~PsafLocalPlanner()
    {
        g_plan_pub.shutdown();
        traffic_situation_sub.shutdown();
        vehicle_status_sub.shutdown();
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
            debug_marker_pub = private_nh.advertise<visualization_msgs::MarkerArray>("debug", 1);
            debug_state_pub = private_nh.advertise<std_msgs::String>("/psaf/debug/local_planner/state", 1);

            global_plan_extended_sub = private_nh.subscribe("/psaf/xroute", 10, &PsafLocalPlanner::globalPlanExtendedCallback, this);
            planner_util.initialize(tf, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
            this->costmap_ros = costmap_ros;

            traffic_situation_sub = private_nh.subscribe("/psaf/local_planner/traffic_situation", 10, &PsafLocalPlanner::trafficSituationCallback, this);
            // Subscribe for vehicle status updates
            this->vehicle_status_sub = private_nh.subscribe("/carla/ego_vehicle/vehicle_status", 10, &PsafLocalPlanner::odometryCallback, this);

            dyn_serv = new dynamic_reconfigure::Server<PsafLocalPlannerParameterConfig>(private_nh);
            dynamic_reconfigure::Server<PsafLocalPlannerParameterConfig>::CallbackType f = boost::bind(&PsafLocalPlanner::reconfigureCallback, this, _1, _2);
            dyn_serv->setCallback(f);

            if (!ros::param::get("respect_traffic_rules",respect_traffic_rules)) {
                respect_traffic_rules = true;
            }

            // Replace the state machine if we drive without traffic rules
            if(respect_traffic_rules==false){
                ROS_WARN("The car will drive without respecting the traffic rules");
                this->state_machine = new LocalPlannerStateMachineWithoutTrafficRules(); // TODO child class isn't called
            }

            this->state_machine->init();

            costmap_raytracer = CostmapRaytracer(costmap_ros, &current_pose, &debug_marker_pub);

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
                max_velocity = 0;
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                goal_reached = true;
            }
            else {

                psaf_messages::XLanelet lanelet_out;
                psaf_messages::CenterLineExtended center_point_out;
                auto target_point = findLookaheadTarget(lanelet_out, center_point_out);

                if (!lanelet_out.isAtIntersection || max_velocity == 0) {
                    if (center_point_out.speed == 0) {
                        max_velocity = global_route[0].route_portion[0].speed / 3.6;
                    } else {
                        max_velocity = std::min(global_route[0].route_portion[0].speed / 3.6,
                                                center_point_out.speed / 3.6);
                    }
                }

                double angle = computeSteeringAngle(target_point, current_pose.pose);

                updateStateMachine();

                target_velocity = getTargetVelDriving();
                // Update target velocity regarding the right of way situation
                if (this->respect_traffic_rules) {
                    target_velocity = getTargetVelIntersectionWithTrafficRules();
                }else{
                    target_velocity = getTargetVelIntersectionWithoutTrafficRules();
                }
                target_velocity = checkLaneChangeFree();

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
        int break_counter = 0;
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
                    break_counter++;
                    if (break_counter > MAX_DELETE_OLD_POINTS_LOOKAHEAD) 
                        break;
                }
            }

            it++;
        }

        // TODO: convert to meter once distance has been added to distance
        deleted_points += to_delete;

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

    // linear interpolation function returns point of fraction t between v0 and v1
    // t=0 would be v0 and t=1 would be v1
    double lerp(double v0, double v1, double t) {
        return (1 - t) * v0 + t * v1;
    }

    void PsafLocalPlanner::globalPlanExtendedCallback(const psaf_messages::XRoute &msg)
    {
        ROS_INFO("RECEIVED MESSAGE: %d", msg.id);

        // validate duration of routes only if a global route already exists
        if (global_route.size() > 0 && !goal_reached && respect_traffic_rules) {
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
        if (global_route.size() > 0 && !goal_reached && !respect_traffic_rules) {
            std::vector<psaf_messages::XLanelet> new_global_route = {};
            new_global_route = msg.route;
            float new_distance = 0;
            float old_distance = 0;
            // calculate distance of new route
            for (auto lanelet : new_global_route) {
                new_distance += lanelet.route_portion[lanelet.route_portion.size()-1].distance;
            }
            // distance of first lanelet of current route
            old_distance += global_route[0].route_portion[global_route[0].route_portion.size()-1].distance;
            // subtract already covered route fragment from distance of whole lanelet
            old_distance -= global_route[0].route_portion[0].distance;
            // calculate remaining distance of current route
            for (int i = 1; i < global_route.size(); i++ ) {
                old_distance += global_route[i].route_portion[global_route[i].route_portion.size()-1].distance;
            }
            // if new route is too long, keep old one
            if (new_distance > (duration_factor * old_distance)){
                ROS_INFO("New Route is at least %f x slower than Current Route -> Current Route is kept", distance_factor);
                return;
            }

        }


        global_route = msg.route;
        goal_reached = false;

        // this block uses linear interpolation to smooth the curves of lanechanges
        // takes the last/first 10 points in both directions and distributes the points along linear line between them
        int size = global_route.size();
        for (int i = 0; i < size; i++) {
            auto &lanelet = global_route[i];
            
            if (lanelet.isLaneChange && i + 1 < size) {
                auto &next_lanelet = global_route[i + 1];
                int lanelet_route_size = lanelet.route_portion.size();
                unsigned long num_points = std::min(std::min(next_lanelet.route_portion.size(), lanelet.route_portion.size()), max_points_smoothing);
                // first point of smoothing
                double x1 = lanelet.route_portion[lanelet_route_size - num_points].x;
                double y1 = lanelet.route_portion[lanelet_route_size - num_points].y;
                // last point of smoothing
                double x2 = next_lanelet.route_portion[num_points - 1].x;
                double y2 = next_lanelet.route_portion[num_points - 1].y;

                // whole lerp is between 0 to 1.0 where 1.0 is at num_points * 2.0
                for (int j = 0; j < num_points; j++) {
                    // half the lerp, between points of the last lanelet
                    // indexing example: 20 - 10 + 9
                    // num points * 2.0 -> num_points at end of current lanelet + num_points at beginning of next lanelet
                    lanelet.route_portion[lanelet_route_size - num_points + j].x = lerp(x1, x2, (j + 1) / (num_points * 2.0));
                    lanelet.route_portion[lanelet_route_size - num_points + j].y = lerp(y1, y2, (j + 1) / (num_points * 2.0));
                }

                for (int j = 0; j < num_points; j++) {
                    // half the lerp, between points of the last lanelet
                    // indexing example: 20 - 10 + 9
                    next_lanelet.route_portion[j].x = lerp(x1, x2, (num_points + j + 1) / (num_points * 2.0));
                    next_lanelet.route_portion[j].y = lerp(y1, y2, (num_points + j + 1) / (num_points * 2.0));
                }
            }
        }


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

        // Reset state machine
        this->state_machine->reset();

        global_plan = points;
        planner_util.setPlan(global_plan);
        publishGlobalPlan(global_plan);
    }

}

