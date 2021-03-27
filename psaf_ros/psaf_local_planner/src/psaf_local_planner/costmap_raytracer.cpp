#include "psaf_local_planner/costmap_raytracer.h"


namespace psaf_local_planner
{

    RaytraceCollisionData::RaytraceCollisionData(double x, double y, double angle, double distance, ros::Time timestamp)
                    : x(x), y(y), angle(angle), distance(distance), timestamp(timestamp)
    {}

    CostmapRaytracer::CostmapRaytracer() 
                    : costmap_ros(nullptr), current_pose(nullptr), debug_pub(nullptr)
    {}

    CostmapRaytracer::CostmapRaytracer(costmap_2d::Costmap2DROS *costmap_ros, geometry_msgs::PoseStamped *current_pose, ros::Publisher *debug_pub) 
                    : costmap_ros(costmap_ros), current_pose(current_pose), debug_pub(debug_pub)
    {}


    void CostmapRaytracer::raytraceSemiCircle(double angle, double distance, std::vector<RaytraceCollisionData> &collisions) {
        raytraceSemiCircle(-angle / 2, angle / 2, distance, collisions);
    }

    void CostmapRaytracer::raytraceSemiCircle(double angle_from, double angle_to, double distance, std::vector<RaytraceCollisionData> &collisions) {
        tf2::Transform current_transform;
        tf2::convert(current_pose->pose, current_transform);

        double m_self_x = current_pose->pose.position.x;
        double m_self_y = current_pose->pose.position.y;

        // Rotation around z axis of the car
        double orientation = tf2::getYaw(current_transform.getRotation());
        ros::Time now = ros::Time::now();

        for (double actual_angle = angle_from; actual_angle <= angle_to; actual_angle += (M_PI / 180) * 5) {
            double x = std::cos(actual_angle + orientation) * distance;
            double y = std::sin(actual_angle + orientation) * distance;
            double coll_x, coll_y;

            double dist = raytrace(x + m_self_x , y + m_self_y, coll_x, coll_y);
            if (dist < INFINITY) {
                collisions.push_back(RaytraceCollisionData(coll_x, coll_y, actual_angle, distance, now));
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
        marker1.scale.x = 0.5;
        marker1.scale.y = 0.5;
        marker1.scale.z = 4;

        for (auto &pos : collisions) {
            geometry_msgs::Point point;
            point.x = pos.x;
            point.y = pos.y;
            point.z = 0;
            marker1.points.push_back(point);
        }

        markers.markers.push_back(marker1);
        debug_pub->publish(markers);
    }

    double CostmapRaytracer::raytrace(double m_target_x, double m_target_y, double &coll_x, double &coll_y) {
        double m_self_x, m_self_y;
        unsigned int c_self_x, c_self_y, c_target_x, c_target_y;
        auto costmap = costmap_ros->getCostmap();

        m_self_x = current_pose->pose.position.x;
        m_self_y = current_pose->pose.position.y;

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

    bool isCollisionInVector(const RaytraceCollisionData &coll, const std::vector<RaytraceCollisionData> &vec, double manhattan_epsilon) {
        for (auto element : vec) {
            if (std::abs(element.x - coll.x) < manhattan_epsilon || std::abs(element.y - coll.y) < manhattan_epsilon)
                return true;
        }

        return false;
    }

    bool CostmapRaytracer::checkForNoMovement(double angle, double distance, unsigned int required_confidence) {
        auto now = ros::Time::now();
        if ((now - last_movement_check) > ros::Duration(MOVEMENT_CHECK_MAX_ITERATION_PAUSE_SECONDS)) {
            confidence = 0;
            last_raytrace_results.clear();
        }


        std::vector<RaytraceCollisionData> raytrace_results;
        raytraceSemiCircle(angle, distance, raytrace_results);
        ros::Time x_sec_before = now - ros::Duration(MOVEMENT_CHECK_SECONDS_TO_KEEP);

        std::remove_if(raytrace_results.begin(), raytrace_results.end(), [](RaytraceCollisionData x){return x.distance < MOVEMENT_CHECK_MIN_DISTANCE;});
        std::remove_if(last_raytrace_results.begin(), last_raytrace_results.end(), [&](RaytraceCollisionData x){return x.timestamp < x_sec_before;});

        if (raytrace_results.empty()) return true;
        
        bool hasMovement = false;


        for (auto coll : raytrace_results) {
            if (!isCollisionInVector(coll, last_raytrace_results, MANHATTAN_EPSILON)) {
                hasMovement = true;
                break;
            }
        }

        last_raytrace_results.insert(last_raytrace_results.end(), raytrace_results.begin(), raytrace_results.end());

        if (hasMovement) {
            confidence = 0;
        } else {
            confidence = std::max(confidence + 1, required_confidence);
        }

        return confidence >= required_confidence;
    }

}