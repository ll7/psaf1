#include "psaf_local_planner/plugin_local_planner.h"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(psaf_local_planner::PsafLocalPlanner, nav_core::BaseLocalPlanner)

namespace psaf_local_planner
{
    PsafLocalPlanner::PsafLocalPlanner() : odom_helper("/carla/ego_vehicle/odometry"), global_plan({}),
                                            initialized(false), closest_point_local_plan(2),
                                            lookahead_factor(0.25), lookahead_factor_exp(1.4), lookahead_factor_const_additive(1.5), 
                                            goal_reached(false), estimate_curvature_distance(50), check_collision_max_distance(40),
                                            slow_car_ahead_counter(0), slow_car_ahead_published(false), obstacle_msg_id_counter(0), 
                                            duration_factor(2.0), distance_factor(2.0), respect_traffic_rules(true), max_points_smoothing(10), 
                                            lane_change_direction(0), lane_change_direction_calculated(false), target_velocity(15), min_velocity(5)
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
                this->state_machine = new LocalPlannerStateMachineWithoutTrafficRules();
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

                    if (!respect_traffic_rules) {
                        max_velocity *= 1.5;
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
     * 
     * 
     * Finds the closest point to the car with a lookahead of MAX_DELETE_OLD_POINTS_LOOKAHEAD points.
     * This means if the closest point is more than MAX_DELETE_OLD_POINTS_LOOKAHEAD up until the car position 
     * while the first point is closer it will not find the closest point
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
                break_counter = 0;
            } else {
                // ensure that the closest point is ahead of the car when it is inside the bounds of the car
                if (distance_sq < closest_point_local_plan_sq) {
                    closest_it = it;
                    to_delete++;
                } else {
                    break_counter++;
                    // prevent checking the whole list, as it is very unlikely that we will find anything after a few hundret points of increasing distance
                    if (break_counter > MAX_DELETE_OLD_POINTS_LOOKAHEAD) 
                        break;
                }
            }

            it++;
        }

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
    }

    // linear interpolation function returns point of fraction t between v0 and v1
    // t=0 would be v0 and t=1 would be v1
    double lerp(double v0, double v1, double t) {
        return (1 - t) * v0 + t * v1;
    }

    
    /**
     * Callback for new Global Plan
     * */
    void PsafLocalPlanner::globalPlanExtendedCallback(const psaf_messages::XRoute &msg)
    {
        ROS_INFO("RECEIVED MESSAGE: %d with lanes %lu", msg.id, msg.route.size());

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

        ROS_INFO("LOCAL_PLANNER: validated new plan.");


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
                int next_lanelet_route_size = next_lanelet.route_portion.size();

                if (lanelet_route_size == 0 || next_lanelet_route_size == 0) continue;

                // smoothing should be dependent on max velocity
                float speed_factor;
                // speed > 50 -> out of town (e.g. Highway) higher factor needed
                if (lanelet.route_portion[0].speed > 50) {
                    speed_factor = lanelet.route_portion[0].speed / 3.6 / 5;
                }
                // speed =< 50 -> inside of town smaller factor for not cutting other lanelets
                else {
                    speed_factor = lanelet.route_portion[0].speed / 3.6 / 10;
                }
                // number of points on current lanelet
                unsigned long num_points_current = std::min(lanelet.route_portion.size(), (long unsigned int)(max_points_smoothing * speed_factor));
                // number of points on next lanelet
                unsigned long num_points_next = std::min(next_lanelet.route_portion.size(), (long unsigned int)(max_points_smoothing * speed_factor));
                // total number of points for smoothing
                double num_points_total = num_points_next + num_points_current;
                // first point of smoothing
                double x1 = lanelet.route_portion[lanelet_route_size - num_points_current].x;
                double y1 = lanelet.route_portion[lanelet_route_size - num_points_current].y;
                // last point of smoothing
                double x2 = next_lanelet.route_portion[num_points_next - 1].x;
                double y2 = next_lanelet.route_portion[num_points_next - 1].y;

                // whole lerp is between 0 to 1.0 where 1.0 is at num_points * 2.0
                for (int j = 0; j < num_points_current; j++) {
                    // half the lerp, between points of the last lanelet
                    // indexing example: 20 - 10 + 9
                    // num points * 2.0 -> num_points at end of current lanelet + num_points at beginning of next lanelet
                    lanelet.route_portion[lanelet_route_size - num_points_current + j].x = lerp(x1, x2, (j + 1) / (num_points_total));
                    lanelet.route_portion[lanelet_route_size - num_points_current + j].y = lerp(y1, y2, (j + 1) / (num_points_total));
                }

                for (int j = 0; j < num_points_next; j++) {
                    // half the lerp, between points of the last lanelet
                    // indexing example: 20 - 10 + 9
                    next_lanelet.route_portion[j].x = lerp(x1, x2, (num_points_current + j + 1) / (num_points_total));
                    next_lanelet.route_portion[j].y = lerp(y1, y2, (num_points_current + j + 1) / (num_points_total));
                }
            }
        }

        ROS_INFO("LOCAL_PLANNER: interpolated lanechanges.");

        const int lookahead_left_turn = 20;
        const float lookahead_left_turn_angle_threshhold = 45 * M_PI/180;

        // When at an intersection and turning left --> inserting a fake stop sign to check for collision
        for (int i = 0; i < size; i++) {
            auto &lanelet = global_route[i];
            if (i + 1 < size) {
                auto &next_lanelet = global_route[i + 1];
                int lanelet_route_size = lanelet.route_portion.size();
                int next_lanelet_route_size = next_lanelet.route_portion.size();

                if (next_lanelet.isAtIntersection && !lanelet.isAtIntersection && !lanelet.hasStop && !lanelet.hasLight) {
                    if (lanelet_route_size > 2 && next_lanelet_route_size > lookahead_left_turn) {
                        // calculate angle between three points
                        auto &last = lanelet.route_portion[lanelet.route_portion.size() - 1];
                        auto &second_last = lanelet.route_portion[lanelet.route_portion.size() - 2];
                        auto &next = next_lanelet.route_portion[lookahead_left_turn];

                        auto v_last = tf2::Vector3(last.x, last.y, last.z);
                        auto v_second_last = tf2::Vector3(second_last.x, second_last.y, second_last.z);
                        auto v_next = tf2::Vector3(next.x, next.y, next.z);

                        auto v1 = v_last - v_second_last;
                        auto v2 = v_next - v_last;

                        double angle = atan2(v2.getY(), v2.getX()) - atan2(v1.getY(), v1.getX());

                        // check if we are changing direction to left:
                        ROS_WARN("Angle: %f", angle);
                        if (angle > lookahead_left_turn_angle_threshhold) {
                            ROS_WARN("Adding a fake stop sign at due left turn at lanechange %i -> %i, at [%f %f]", lanelet.id, next_lanelet.id, next_lanelet.route_portion[0].x, next_lanelet.route_portion[0].y);
                            lanelet.hasStop = true;
                        }
                    }
                }
            }
        }

        ROS_INFO("LOCAL_PLANNER: fake stop check done.");



        // Convert xroute back into normal pose stamp list
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
        publishAdditionalInfoToRviz();

        ROS_INFO("PREPROCESSING XROUTE DONE!");
    }


    void PsafLocalPlanner::publishAdditionalInfoToRviz() {
        // Publish stop signs as markers 
        auto markers = visualization_msgs::MarkerArray();
        auto marker_stop = visualization_msgs::Marker();
        auto marker_traffic = visualization_msgs::Marker();

        marker_stop.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_stop.action = visualization_msgs::Marker::ADD;
        marker_stop.ns = "stop";
        marker_stop.header.frame_id = "map";
        marker_stop.header.stamp = ros::Time::now();
        marker_stop.color.a = 1.0;
        marker_stop.color.r = 1.0;
        marker_stop.color.g = 1.0;
        marker_stop.scale.x = 2;
        marker_stop.scale.y = 2;
        marker_stop.scale.z = 4;

        marker_traffic.type = visualization_msgs::Marker::SPHERE_LIST;
        marker_traffic.action = visualization_msgs::Marker::ADD;
        marker_traffic.ns = "traffic";
        marker_traffic.header.frame_id = "map";
        marker_traffic.header.stamp = ros::Time::now();
        marker_traffic.color.a = 1.0;
        marker_traffic.color.g = 1.0;
        marker_traffic.scale.x = 2;
        marker_traffic.scale.y = 2;
        marker_traffic.scale.z = 4;

        double last_vel = 0;
        int speed_counter = 0;
        for (auto &lanelet : global_route) {
            if (lanelet.hasStop) {
                geometry_msgs::Point point;
                point.x = lanelet.route_portion[lanelet.route_portion.size() - 1].x;
                point.y = lanelet.route_portion[lanelet.route_portion.size() - 1].y;
                point.z = 0;
                marker_stop.points.push_back(point);
            }

            if (lanelet.hasLight) {
                geometry_msgs::Point point;
                point.x = lanelet.route_portion[lanelet.route_portion.size() - 1].x;
                point.y = lanelet.route_portion[lanelet.route_portion.size() - 1].y;
                point.z = 0;
                marker_traffic.points.push_back(point);
            }

            bool start = true;
            for (auto &center : lanelet.route_portion) {
                if (abs(center.speed - last_vel) > 0.001 || start) {
                    auto marker_speed = visualization_msgs::Marker();
                    marker_speed.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    marker_speed.action = visualization_msgs::Marker::ADD;
                    marker_speed.ns = "speed";
                    marker_speed.id = speed_counter++;
                    marker_speed.header.frame_id = "map";
                    marker_speed.header.stamp = ros::Time::now();
                    marker_speed.color.a = 1.0;
                    marker_speed.color.r = 1.0;
                    marker_speed.color.g = 1.0;
                    marker_speed.color.b = 1.0;
                    marker_speed.pose.position.x = center.x;
                    marker_speed.pose.position.y = center.y;
                    marker_speed.scale.z = 2;
                    if (start) {
                        marker_speed.text = "ID" + std::to_string(lanelet.id) + " | S" + std::to_string(center.speed);
                    } else {
                        marker_speed.text = "S" + std::to_string(center.speed);
                    }
                    markers.markers.push_back(marker_speed);

                    last_vel = center.speed;
                    start = false;
                }
            }
        }

        markers.markers.push_back(marker_stop);
        markers.markers.push_back(marker_traffic);
        debug_marker_pub.publish(markers);

        
        ROS_INFO("LOCAL_PLANNER: done publishing additional info.");
    }

}

