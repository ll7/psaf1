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
        debug_marker_pub.publish(markers);

        return d_angle;
    }
    /**
     * function to calculate radius with the use of menger curvature
     * https://en.wikipedia.org/wiki/Menger_curvature#Definition
     */
    double PsafLocalPlanner::calculateRadius(unsigned int first, unsigned int last) {
        // no curvature therefore radius -> infinity
        if (first == last || last == 0) {
            return INFINITY;
        }
        // curvature too small
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



        // https://math.stackexchange.com/questions/516219/finding-out-the-area-of-a-triangle-if-the-coordinates-of-the-three-vertices-are
        double triangle_area = ((x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1)) / 2.0;
        // https://stackoverflow.com/questions/41144224/calculate-curvature-for-3-points-x-y
        double menger = (4 * triangle_area)/(tf2::tf2Distance(p1, p2)*tf2::tf2Distance(p2, p3)*tf2::tf2Distance(p3, p1));
        double r_m = std::abs(1.0 / menger);

        return r_m;
    }
    /**
     * function to calculate target velocity depending on the curvature
     * */
    double PsafLocalPlanner::estimateCurvatureAndSetTargetVelocity(){
        // calculation of curvature only possible with 3 or more points
        if (global_plan.size() < 3)
            return target_velocity;

        tf2::Vector3 point1, point2, point3;
        // index of the points that we want to compare later
        unsigned int first = 0, last = 0;

        // Whether the algorithm has found an angle which is != 0
        bool hasNonZero = false;

        auto it = global_plan.begin();

        // beginn with getting the first two points to init the loop correctly
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

        // iterate over global plan and check next {estimate_curvature_distance} meters
        // tries to find the smallest circle in the upcoming road
        for (; it != global_plan.end(); ++it, ++last)
        {
            // stop when we checked the required distance
            if (sum_distance > estimate_curvature_distance)
                break;

            const geometry_msgs::PoseStamped &w = *it;
            tf2::convert(w.pose.position, point3);

            sum_distance += tf2::tf2Distance(point2, point3);
            auto v1 = (point2 - point1);
            auto v2 = (point3 - point2);
            
            // Always calculate angle of the last three points
            double angle = v1.angle(v2);
            
            // With this we can figure out whether we are on a straight road or heading into a curve
            if (isfinite(angle)) {
                // intended purpose of this:
                // find three points on the curvature
                sum_angle += abs(angle);

                // we are currently checking a straight line
                if (abs(angle) < 0.0001) {
                    // find first point on circle; hasNonZero == false means that we are on a straight line before we found any curve
                    if (!hasNonZero) {
                        first++;
                    } else {
                        // curvature has ended; calculate raidus for the given points
                        min_radius = std::min(min_radius, calculateRadius(first, last));
                        first = last;
                        hasNonZero = false;
                    }
                // we are in a curce right now
                } else {
                    hasNonZero = true;
                    // curvature is bending in different direction, therefore ended as well
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

        // no curverature therefore no velocity restriction
        if (min_radius == INFINITY) {
            return getMaxVelocity();
        }

        // max speed in curves: v <= sqrt(µ_haft * r * g)
        // µ_haft ~= 0.8 - 1.0
        double target_vel = std::min(getMaxVelocity(), std::sqrt(0.8 * min_radius * 9.81));
        // ROS_INFO("radius: %f, target vel: %f", min_radius, target_vel);
        return target_vel;
    }

    /** 
     * Finds the next target point along the global plan using the lookahead_factor and lookahead distance
     */
    geometry_msgs::Pose PsafLocalPlanner::findLookaheadTarget(psaf_messages::XLanelet &lanelet_out, psaf_messages::CenterLineExtended &center_point_out) {
        tf2::Vector3 last_point, current_point, acutal_point;
        tf2::convert(current_pose.pose.position, last_point);
        last_point.setZ(0);
        acutal_point = last_point;

        geometry_msgs::PoseStamped vel;
        odom_helper.getRobotVel(vel);

        double vel_x = std::ceil(std::abs(vel.pose.position.x));

        double desired_distance = std::pow(vel_x * lookahead_factor, lookahead_factor_exp) + lookahead_factor_const_additive;
        double sum_distance = 0;

        // count up to the given distance and return the new pose
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


        // Special treatment if we reached the end when we didn't reach a point in the desired distance
        // Returns the last possible point on the list (with few empty checks)
        geometry_msgs::Pose last_stamp; 
        if (global_route.size() > 0 && global_plan.size() > 0) {
            lanelet_out = *global_route.end();
            if (lanelet_out.route_portion.size() > 0) {
                center_point_out = *lanelet_out.route_portion.end();
            }
            last_stamp = (*global_plan.end()).pose;
        }

        return last_stamp;
    }

    double PsafLocalPlanner::getMaxVelocity() {
        return max_velocity;
    }

    double PsafLocalPlanner::getCurrentStoppingDistance(){
        return (pow(this->current_speed,2)*0.1296+1.2 * current_speed); // Standard formula with extra reaction time
    }


    double PsafLocalPlanner::getTargetVelocityForDistance(double distance) {
        // slow and safe formula, working good
        // uses formula for Anhalteweg (solved for velocity instead of distance)
        // https://www.bussgeldkatalog.org/anhalteweg/
        // faster formula, requires faster controller
        // velocity_distance_diff = target_vel - std::min(target_velocity, 25.0/9.0 * std::sqrt(distance - 5));
        return 25.0/18.0 * (-1 + std::sqrt(1 + 4 * (distance - 5)));
    }

    /**
     * function to adjust target velocity according to obstacles ahead
     * */
    double PsafLocalPlanner::getTargetVelDriving()
    {
        double target_vel = estimateCurvatureAndSetTargetVelocity();
        double distance, relX, relY;
        double velocity_distance_diff;

        if (target_vel > 0 && !checkDistanceForward(distance, relX, relY))
        {
            // Alawys hard stop 5m behind any vehicle as it is to the center of our vehicle
            if (distance < 5)
            {
                velocity_distance_diff = target_vel;
            } else {
                // slowly drive near the obstacle
                velocity_distance_diff = target_vel - std::min(target_vel, getTargetVelocityForDistance(distance));
            }

            ROS_DEBUG("distance forward: %f, max velocity: %f", distance, target_vel);
        }
        
        // Check for slow car: Initiate a lanechange if it falls below a threshhold
        checkForSlowCar(velocity_distance_diff);

        target_vel = target_vel - velocity_distance_diff;
        return target_vel;
    }
    /**
     * function to calculate the distance to the next intersection
     */
    double PsafLocalPlanner::getDistanceToIntersection() {
        double distance = 0;
        // iterate over upcoming lanelets in the route
        for (auto &lanelet : global_route) {
            // sum up distance until lanelet is marked as intersection
            distance += lanelet.route_portion.end()->distance - lanelet.route_portion.begin()->distance;
            if (lanelet.isAtIntersection)
                return distance;
        }

        return distance;
    }
    /**
     * function to check if area of lane change is free and return speed accordingly
     */
    double PsafLocalPlanner::checkLaneChangeFree() {
        // distance treshold to lane change from where further needed calculation ist done
        double distance_begin_check_lane_change = 20;
        // radius of area to check for obstacles when lane changing
        double check_distance_lanechange = 5;
        // calc distance to next alne change
        double distance = getDistanceToLaneChange(distance_begin_check_lane_change);

        if (distance < distance_begin_check_lane_change) {
            // lanechange ahead, but lane change direction not calculated therefore presume as before
            if (lane_change_direction == 0) {
                ROS_WARN("Direction is 0, but we should check for collsion! Driving normally");
                return target_velocity;
            }

            std::vector<RaytraceCollisionData> collisions = {};
            // calculate angles for ray tracing circle area
            double angle_from, angle_to;
            // right(1) or left(-1)
            if (lane_change_direction <= 0) {
                angle_from = M_PI / 8.0;
                angle_to = M_PI * (3.0/4.0);
            } else {
                angle_from = 2 * M_PI - M_PI/8;
                angle_to = M_PI + M_PI/4;
            }

            // raytrace area
            costmap_raytracer.raytraceSemiCircle(angle_from, angle_to, check_distance_lanechange, collisions);
            // set max velocity according to ANHALTEWEG if obstacle in area
            if (collisions.size() > 0) {
                return std::min(target_velocity, getTargetVelocityForDistance(distance));
            }
        }

        return target_velocity;
    }

    /**
     * function to calculate the distance to the next lanechange
     * */
    double PsafLocalPlanner::getDistanceToLaneChange(double compute_direction_threshold) {
        // within the distance of the compute_direction_treshold
        double distance = 0;

        // iterate over the lanelets of the global route
        for (int i = 0; i < global_route.size(); i++) {
            auto &lanelet = global_route[i];
            distance += lanelet.route_portion[lanelet.route_portion.size() - 1].distance - lanelet.route_portion[0].distance;

            // LaneChange in Lanelet is existant
            if (lanelet.isLaneChange) {
                //LanneChange is closer than threshold
                if (compute_direction_threshold >= distance) {
                    lane_change_direction = lanechange_direction_map[lanelet.id];
                    return distance;

                    } else {
                        // We are for enough away so that we don't care anymore;
                        return INFINITY;
                    }
                // recent LaneChang is terminated, reset flags
                } else {
                    lane_change_direction_calculated = false;
                    lane_change_direction = 0;
                }
            }

        return distance;
    }

}
