#include "psaf_local_planner/plugin_local_planner.h"

namespace psaf_local_planner
{

    /**
     * Compute relative angle and distance between a target_location and a current_location
     */
    double PsafLocalPlanner::computeSteeringAngle(geometry_msgs::Pose target_location, geometry_msgs::Pose current_location)
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

    double PsafLocalPlanner::calculateRadius(unsigned int first, unsigned int last) {
        if (first == last || last == 0) {
            return INFINITY;
        }

        if (last - first <= 5) {
            return INFINITY;
        }

        unsigned int middle = first + (last - first) / 2;
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
        double r_m = std::abs(1.0 / menger);


        return r_m;
    }

    double PsafLocalPlanner::estimateCurvatureAndSetTargetVelocity()
    {
        if (global_plan.size() < 3)
            return target_velocity;

        tf2::Vector3 point1, point2, point3;
        unsigned int first = 0, middle, last = 0;
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

        double min_radius = INFINITY;

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
                        min_radius = std::min(min_radius, calculateRadius(first, last));
                        first = last;
                        hasNonZero = false;
                    }
                } else {
                    hasNonZero = true;
                    // curvature is bending in different direction, therefor ended as well
                    if (std::signbit(last_angle) != std::signbit(angle)) {
                        min_radius = std::min(min_radius, calculateRadius(first, last));
                        first = last;
                        hasNonZero = false;
                    }
                }

                last_angle = angle;
            }

            point1 = point2;
            point2 = point3;
        }


        if (min_radius == INFINITY) {
            return getMaxVelocity();
        }

        // max speed in curves: v <= sqrt(µ_haft * r * g)
        // µ_haft ~= 0.8 - 1.0
        double target_vel = std::min(getMaxVelocity(), std::sqrt(0.8 * min_radius * 9.81));
        ROS_INFO("radius: %f, target vel: %f", min_radius, target_vel);
        return target_vel;
    }

    geometry_msgs::Pose PsafLocalPlanner::findLookaheadTarget(psaf_messages::XLanelet &lanelet_out, psaf_messages::CenterLineExtended &center_point_out) {
        tf2::Vector3 last_point, current_point, acutal_point;
        tf2::convert(current_pose.pose.position, last_point);
        last_point.setZ(0);
        acutal_point = last_point;
        ROS_INFO("Beginn");

        geometry_msgs::PoseStamped vel;
        odom_helper.getRobotVel(vel);

        double vel_x = std::ceil(std::abs(vel.pose.position.x));

        double desired_distance = std::pow(vel_x / lookahead_factor, 1.1) + 5;
        double sum_distance = 0;

        for (auto &lanelet : global_route) {
            double base_dist = lanelet.route_portion[0].distance;
            for (auto &point : lanelet.route_portion) {
                double dist = sum_distance + point.distance - base_dist;
                if (dist > desired_distance) {
                    geometry_msgs::Pose pose;
                    pose.position.x = point.x;
                    pose.position.y = point.y;

                    lanelet_out = lanelet;
                    center_point_out = point;

                    return pose;
                }
            }

            sum_distance += (*lanelet.route_portion.end()).distance - base_dist;
        }

        /*for (auto it = global_plan.begin(); it != global_plan.end(); ++it)
        {
            geometry_msgs::PoseStamped &w = *it;
            tf2::convert(w.pose.position, current_point);
            sum_distance += tf2::tf2Distance(last_point, current_point);

            if (sum_distance > desired_distance) {
                return w;
            }

            last_point = current_point;
        }*/

        ROS_INFO("Before");
        geometry_msgs::Pose last_stamp; 
        if (global_route.size() > 0 && global_plan.size() > 0) {
            lanelet_out = *global_route.end();
            if (lanelet_out.route_portion.size() > 0) {
                center_point_out = *lanelet_out.route_portion.end();
            }
            last_stamp = (*global_plan.end()).pose;
        }

        
        ROS_INFO("After");

        return last_stamp;
    }

    double PsafLocalPlanner::getMaxVelocity() {
        return max_velocity;
    }

    double PsafLocalPlanner::getCurrentStoppingDistance(){
        return pow(this->current_speed,2)*0.1296+current_speed;
    }

    double PsafLocalPlanner::getTargetVelDriving()
    {
                double target_vel = estimateCurvatureAndSetTargetVelocity();
                double distance, relX, relY;
                double velocity_distance_diff;

                if (target_vel > 0 && !checkDistanceForward(distance, relX, relY))
                {
                    if (distance < 5)
                    {
                        ROS_INFO("attempting to stop");
                        velocity_distance_diff = target_vel;
                    } else {
                        // TODO: validate if working
                        // slow formula, working okay ish
                        // uses formula for Anhalteweg (solved for velocity instead of distance)
                        // https://www.bussgeldkatalog.org/anhalteweg/
                        // TODO: MOVE TO OWN FUNCTION
                        velocity_distance_diff = target_vel - std::min(target_vel, 25.0/18.0 * (-1 + std::sqrt(1 + 4 * (distance - 5))));
                        // faster formula, requires faster controller
                        //velocity_distance_diff = target_vel - std::min(target_velocity, 25.0/9.0 * std::sqrt(distance - 5));
                    }

                    ROS_INFO("distance forward: %f, max velocity: %f", distance, target_vel);
                }

                checkForSlowCar(velocity_distance_diff);

                target_vel = target_vel - velocity_distance_diff;
                return target_vel;
    }

    double PsafLocalPlanner::getDistanceToIntersection() {
        double distance = 0;

        for (auto lanelet : global_route) {
            distance += lanelet.route_portion[lanelet.route_portion.size() - 1].distance - lanelet.route_portion[0].distance;
            if (lanelet.isAtIntersection)
                return distance;
        }

        return distance;
    }

    double PsafLocalPlanner::checkLaneChangeFree() {
        int direction = 0;
        double distance_begin_check_lane_change = 10;
        double check_distance_lanechange = 7;

        double distance = getDistanceToLaneChange(distance_begin_check_lane_change * 2, direction);

        if (distance < distance_begin_check_lane_change) {
            if (direction == 0) {
                ROS_WARN("Direction is 0, but we should check for collsion! Driving normally");
                return target_velocity;
            }

            std::vector<RaytraceCollisionData> collisions = {};

            double angle_from, angle_to;
            if (direction > 0) {
                angle_from = M_PI / 4.0;
                angle_to = M_PI * (3.0/4.0);
            } else {
                angle_to = -M_PI / 4.0;
                angle_from = -M_PI * (3.0/4.0);
            }

            costmap_raytracer.raytraceSemiCircle(angle_from, angle_to, check_distance_lanechange, collisions);

            if (collisions.size() > 0) {
                // TODO: MOVE TO OWN FUNCTION
                return std::min(target_velocity, 25.0/18.0 * (-1 + std::sqrt(1 + 4 * (distance - 5))));
            }
        }

        return target_velocity;
    }

    double PsafLocalPlanner::getDistanceToLaneChange(double compute_direction_threshold, int &direction) {
        double distance = 0;

        for (int i = 0; i < global_route.size(); i++) {
            auto &lanelet = global_route[i];
            distance += lanelet.route_portion[lanelet.route_portion.size() - 1].distance - lanelet.route_portion[0].distance;

            if (lanelet.isLaneChange) {
                if (compute_direction_threshold >= distance) {
                    if (i + 1 < global_route.size()) {
                        auto &next_lanelet = global_route[i + 1];
                        if (lanelet.route_portion.size() >= 2) {
                            auto &last = lanelet.route_portion[lanelet.route_portion.size() - 1];
                            auto &second_last = lanelet.route_portion[lanelet.route_portion.size() - 2];
                            auto &next = next_lanelet.route_portion[0];

                            auto v_last = tf2::Vector3(last.x, last.y, last.z);
                            auto v_second_last = tf2::Vector3(second_last.x, second_last.y, second_last.z);
                            auto v_next = tf2::Vector3(next.x, next.y, next.z);

                            auto v1 = v_last - v_second_last;
                            auto v2 = v_next - v_last;

                            double angle = atan2(v2.getY(), v2.getX()) - atan2(v1.getY(), v1.getX());

                            if (angle > 0) {
                                direction = +1;
                            } else {
                                direction = -1;
                            }

                        } else {
                            ROS_WARN("Not enough points to use three point method");
                        }
                    } else {
                        ROS_ERROR("LANECHANGE MARKED WITHOUT SUCCESING LANELET! CALL GLOBAL PLANNER SUPPORT!");
                    }
                }
                

                return distance;
            }
        }

        return distance;
    }
}