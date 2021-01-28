#include "psaf_local_planner/plugin_local_planner.h"


namespace psaf_local_planner
{

    RaytraceCollisionData::RaytraceCollisionData(double x, double y, double angle, double distance)
            : x(x), y(y), angle(angle), distance(distance)
    {}

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
}