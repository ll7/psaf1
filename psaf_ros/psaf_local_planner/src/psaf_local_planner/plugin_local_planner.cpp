// #include "psaf_steering/plugin_local_planner.h"
#include "psaf_local_planner/plugin_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <boost/algorithm/clamp.hpp>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(psaf_local_planner::PsafLocalPlanner, nav_core::BaseLocalPlanner)

namespace psaf_local_planner
{
    PsafLocalPlanner::PsafLocalPlanner() : odom_helper("/carla/ego_vehicle/odometry"), global_plan({}),
                                           bufferSize(1000), initialized(false), closest_point_local_plan(2),
                                           lookahead_factor(3), max_velocity(30), target_velocity(30), min_velocity(5),
                                           goal_reached(false), estimate_curvature_distance(30), check_collision_max_distance(40), 
                                           slow_car_ahead_counter(0), slow_car_ahead_published(false), obstacle_msg_id_counter(0)
    {
        std::cout << "Hi";
    }

    void PsafLocalPlanner::reconfigure_callback(psaf_local_planner::PsafLocalPlannerParameterConfig &config, uint32_t level) {
       ROS_INFO("Reconfigure Request");
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

    PsafLocalPlanner::~PsafLocalPlanner()
    {
        g_plan_pub.shutdown();
        vel_sub.shutdown();
        delete dyn_serv;
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

    geometry_msgs::PoseStamped& PsafLocalPlanner::find_lookahead_target() {
        tf2::Vector3 last_point, current_point, acutal_point;
        tf2::convert(current_pose.pose.position, last_point);
        last_point.setZ(0);
        acutal_point = last_point;

        geometry_msgs::PoseStamped vel;
        odom_helper.getRobotVel(vel);

        // ROS_INFO("velx: %f y: %f", vel.pose.position.x, vel.pose.position.y);


        double desired_distance = std::pow(vel.pose.position.x / lookahead_factor, 1.2) + 5;
        double sum_distance = 0;

        for (auto it = global_plan.begin(); it != global_plan.end(); ++it)
        {
            geometry_msgs::PoseStamped &w = *it;
            tf2::convert(w.pose.position, current_point);
            sum_distance += tf2::tf2Distance(last_point, current_point);

            if (sum_distance > desired_distance) {
                return w;
            }

            last_point = current_point;
        }

        auto &last_stamp = *global_plan.end();
        return last_stamp;
    }

    void PsafLocalPlanner::checkForSlowCar(double velocity_distance_diff) {
        // Check if there should be a line change if speed is slower than x for y iterations
        if (velocity_distance_diff > 5) {
            slow_car_ahead_counter = std::min(50, slow_car_ahead_counter + 1);
        } else {
            slow_car_ahead_counter = std::max(0, slow_car_ahead_counter - 2);
        }

        ROS_INFO("slow car counter: %d", slow_car_ahead_counter);

        if (slow_car_ahead_counter > 30 && (!slow_car_ahead_published || ros::Time::now() - slow_car_last_published > ros::Duration(3.0))) {
            ROS_INFO("publishing obstacle ahead");
            slow_car_ahead_published = true;
            slow_car_last_published = ros::Time::now();
            

            std::vector<RaytraceCollisionData> collisions = {};
            raytraceSemiCircle(M_PI * 3/2, 30, collisions);

            std::vector<geometry_msgs::Point> points = {};
            
            for (auto &pos : collisions) {
                geometry_msgs::Point p;
                p.x = pos.x;
                p.y = pos.y;
                p.z = 0;

                ROS_INFO("collsions: %f %f", p.x, p.y);

                points.push_back(p);
            }

            psaf_messages::Obstacle msg;
            msg.id = obstacle_msg_id_counter++;
            // msg.id = points.size();
            msg.obstacles = points;

            obstacle_pub.publish(msg);
        }

        if (slow_car_ahead_counter < 10 && slow_car_ahead_published) {
            ROS_INFO("publishing loss of obstacle");
            slow_car_ahead_published = false;

            psaf_messages::Obstacle msg;
            msg.id = obstacle_msg_id_counter++;
            msg.obstacles = {};

            obstacle_pub.publish(msg);
        }
    }

    void PsafLocalPlanner::raytraceSemiCircle(double angle, double distance, std::vector<RaytraceCollisionData> &collisions) {
        tf2::Transform current_transform;
        tf2::convert(current_pose.pose, current_transform);

        double m_self_x = current_pose.pose.position.x;
        double m_self_y = current_pose.pose.position.y;

        // std::vector<std::tuple<double, double>> collisions = {};

        // Rotation around z axis of the car
        double orientation = tf2::getYaw(current_transform.getRotation());
        
        for (double actual_angle = -angle / 2; actual_angle <= angle / 2; actual_angle += (M_PI / 180) * 10) {
            double x = std::cos(actual_angle + orientation) * distance;
            double y = std::sin(actual_angle + orientation) * distance;
            double coll_x, coll_y;

            double dist = raytrace(x + m_self_x , y + m_self_y, coll_x, coll_y);
            //ROS_INFO("dist at angle: %f at %f, %f is %f", actual_angle, x, y, dist);
            if (dist < INFINITY) {
                collisions.push_back(RaytraceCollisionData(coll_x, coll_y, angle, distance));
                ROS_INFO("dist at angle: %f at %f, %f is %f", actual_angle, x, y, dist);
            }
        }

                // Debug data for the direction and the angle between the car and the road
        auto markers = visualization_msgs::MarkerArray();
        auto marker1 = visualization_msgs::Marker();

        marker1.type = visualization_msgs::Marker::SPHERE_LIST;
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.ns = "obstacle";
        marker1.header.frame_id = "map";
        marker1.header.stamp = ros::Time::now();
        marker1.color.a = 1.0;
        marker1.color.r = 1.0;
        //marker1.pose.position.x = pos.x;
        //marker1.pose.position.y = pos.y;
        marker1.scale.x = 0.5;
        marker1.scale.y = 0.5;
        marker1.scale.z = 4;

        for (auto &pos : collisions) {
            geometry_msgs::Point point;
            point.x = pos.x;
            point.y = pos.y;
            point.z = 0;
            marker1.points.push_back(point);
            ROS_INFO("collision at %f, %f", pos.x, pos.y);
        }
        
        markers.markers.push_back(marker1);
        debug_pub.publish(markers);
    }

    double PsafLocalPlanner::raytrace(double m_target_x, double m_target_y, double &coll_x, double &coll_y) {
        double m_self_x, m_self_y;
        unsigned int c_self_x, c_self_y, c_target_x, c_target_y;
        auto costmap = costmap_ros->getCostmap();

        m_self_x = current_pose.pose.position.x;
        m_self_y = current_pose.pose.position.y;
        
        if (!costmap->worldToMap(m_self_x, m_self_y, c_self_x, c_self_y)) {
            ROS_WARN("self out of bounds: %f %f", m_self_x, m_self_y);
            return INFINITY;
        }

        if (!costmap->worldToMap(m_target_x, m_target_y, c_target_x, c_target_y)) {
            ROS_WARN("target out of bounds: %f %f", m_target_x, m_target_y);
            return INFINITY;
        }

        
        for(base_local_planner::LineIterator line(c_self_x, c_self_y, c_target_x, c_target_y); line.isValid(); line.advance())
        {
            int cost = costmap->getCost(line.getX(), line.getY());
            if (cost > 128 && cost != costmap_2d::NO_INFORMATION) {
                costmap->mapToWorld(line.getX(), line.getY(), coll_x, coll_y);
                return std::sqrt((m_self_x - coll_x) * (m_self_x - coll_x) + (m_self_y - coll_y) * (m_self_y - coll_y));
            }
        }

        return INFINITY;
    }

    /**
     * Check if the goal pose has been achieved by the local planner.
     */
    bool PsafLocalPlanner::isGoalReached()
    {
        return goal_reached && global_plan.size() <= 1;
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

    void PsafLocalPlanner::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (!path.empty())
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
        auto marker3 = visualization_msgs::Marker();
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

        marker3.type = visualization_msgs::Marker::SPHERE;
        marker3.action = visualization_msgs::Marker::ADD;
        marker3.ns = "next";
        marker3.header.frame_id = "map";
        marker3.header.stamp = ros::Time::now();
        marker3.color.a = 1.0;
        marker3.color.r = 0.0;
        marker3.color.g = 1.0;
        marker3.pose = target_location;
        marker3.scale.x = 0.2;
        marker3.scale.y = 0.2;
        marker3.scale.z = 0.2;

        markers.markers = {marker1, marker2, marker3};
        debug_pub.publish(markers);

        return d_angle;
    }

    void PsafLocalPlanner::estimate_curvature_and_set_target_velocity(geometry_msgs::Pose current_location)
    {
        if (global_plan.size() < 3)
            return;

        tf2::Vector3 point1, point2, point3;
        int first = 0, middle, last = 0;
        bool hasNonZero = false;
        
        auto it = global_plan.begin();

        // tf2::convert(current_location.position, point1);
        const geometry_msgs::PoseStamped &w = *it;
        tf2::convert(w.pose.position, point1);
        
        ++it;
        ++last;

        const geometry_msgs::PoseStamped &w2  = *it;
        tf2::convert(w2.pose.position, point2);
        
        ++it;
        ++last;
        double sum_distance = tf2::tf2Distance2(point1, point2);
        double sum_angle = 0;
        double last_angle = 0;


        
        for (; it != global_plan.end(); ++it, ++last)
        {
            if (sum_distance > estimate_curvature_distance) 
                break;

            const geometry_msgs::PoseStamped &w = *it;
            tf2::convert(w.pose.position, point3);

            sum_distance += tf2::tf2Distance(point2, point3);
            auto v1 = (point2 - point1);
            auto v2 = (point3 - point2);

            double angle = v1.angle(v2);
            if (isfinite(angle)) {
                // indendet purpose of this: 
                // find three points on the curvature
                sum_angle += abs(angle);
                if (abs(angle) < 0.0001) {
                    // find first point on circle
                    if (!hasNonZero) {
                        first++;
                    } else {
                        // curvature has ended
                        break;
                    }
                } else {
                    hasNonZero = true;
                    // curvature is bending in different direction, therefor ended as well
                    if (std::signbit(last_angle) != std::signbit(angle)) {
                        break;
                    }
                }

                last_angle = angle;
            }

            point1 = point2;
            point2 = point3;
        }


        if (first == last || last == 0) {
            target_velocity = max_velocity;
            return;
        }

        if (last - first <= 5) {
            target_velocity = max_velocity;
            return;
        }

        middle = first + (last - first) / 2;
        double x1 = global_plan[first].pose.position.x;
        double y1 = global_plan[first].pose.position.y;

        double x2 = global_plan[middle].pose.position.x;
        double y2 = global_plan[middle].pose.position.y;

        double x3 = global_plan[last].pose.position.x;
        double y3 = global_plan[last].pose.position.y;

        auto p1 = tf2::Vector3(x1, y1, 0);
        auto p2 = tf2::Vector3(x2, y2, 0);
        auto p3 = tf2::Vector3(x3, y3, 0);

        // find center point of the circle using http://www.ambrsoft.com/TrigoCalc/Circle3D.htm
        /*double a = x1*(y2-y3)-y1*(x2-x3)+x2*y3-x3*y2;
        
        double b =  (x1*x1+y1*y1)*(y3-y2)+(x2*x2+y2*y2)*(y1-y3)+(x3*x3+y3*y3)*(y2-y1);

        double c = (x1*x1 + y1*y1) * (x2 - x3) + (x2*x2 + y2*y2)*(x3 - x1) + (x3*x3 + y3*y3)*(x1 - x2);
        double d = (x1*x1 + y1*y1)*(x3*y2 - x2*y3) + (x2*x2 + y2*y2)*(x1*y3-x3*y1)+(x3*x3 + y3*y3)*(x2*y1-x1*y2);

        double r = std::sqrt((b * b + c * c - (4 * a * d)) / (4 * a * a));
        double center_x = - b/(2*a);
        double center_y = - c/(2*a);

        ROS_INFO("first: %d, middle: %d, last: %d", first, middle, last);
        ROS_INFO("x1: %f y1: %f x2: %f y2:%f x3: %f y3: %f", x1, y1, x2, y2, x3, y3);
        ROS_INFO("a: %f b: %f c: %f d:%f", a, b, c, d);
        ROS_INFO("r: %f, cx: %f cy: %f", r, center_x, center_y);*/

        // next try: 
        // https://stackoverflow.com/questions/41144224/calculate-curvature-for-3-points-x-y
        // https://en.wikipedia.org/wiki/Menger_curvature#Definition
        // https://math.stackexchange.com/questions/516219/finding-out-the-area-of-a-triangle-if-the-coordinates-of-the-three-vertices-are
        double triangle_area = ((x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1)) / 2.0;
        double menger = (4 * triangle_area)/(tf2::tf2Distance(p1, p2)*tf2::tf2Distance(p2, p3)*tf2::tf2Distance(p3, p1));
        double r_m = 1.0 / menger;

        // Circumference of a circle segment in rad: C = phi * r
        // r = C / phi
        // max speed in curves: v <= sqrt(µ_haft * r * g)
        // µ_haft ~= 0.8 - 1.0
        
        //auto fact = boost::algorithm::clamp(sum_angle * 10  / sum_distance, 0, 1);

        // target_velocity = (max_velocity - fact * (max_velocity - min_velocity));
        target_velocity = std::min(max_velocity, std::sqrt(1.0 * r_m * 9.81));
        ROS_INFO("radius: %f, target vel: %f", r_m, target_velocity);
    }

    /**
     * Returns false if it failed because something is close; true if out of bounds
     */
    bool PsafLocalPlanner::check_distance_forward(double& distance, double &relativeX, double &relativeY)
    {
        tf2::Vector3 last_point, current_point, acutal_point;
        tf2::convert(current_pose.pose.position, last_point);
        last_point.setZ(0);
        acutal_point = last_point;

        double sum_distance = 0;
        int count_error = 0;

        for (auto it = global_plan.begin(); it != global_plan.end(); ++it)
        {
            if (sum_distance > check_collision_max_distance)
                break;

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
                        relativeX = current_point.getX() - acutal_point.getX();
                        relativeY = current_point.getY() - acutal_point.getY();
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


    RaytraceCollisionData::RaytraceCollisionData(double x, double y, double angle, double distance) 
    : x(x), y(y), angle(angle), distance(distance)
    {}
} // namespace psaf_local_planner 