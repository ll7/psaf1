#include "psaf_local_planner/plugin_local_planner.h"


namespace psaf_local_planner
{

    /**
    * Function to check if there should be a obstacle published
    * if speed of car/obstacle ahead is slower than VEL_DIF_THRESHOLD for NUM_SLOW_CAR_PUB iterations
    */
    void PsafLocalPlanner::checkForSlowCar(double velocity_distance_diff) {
        // working with counter that function is not blocking and peaks get normalized
        if (velocity_distance_diff > VEL_DIFF_THRESHOLD) {
            // counter is maxed to 50 to not overflow e.g. when waiting behind vehicle
            slow_car_ahead_counter = std::min(50, slow_car_ahead_counter + 1);
        } else {
            // decrease counter if no slower obstacle ahed
            slow_car_ahead_counter = std::max(0, slow_car_ahead_counter - 2);
        }

        // ROS_INFO("slow car counter: %d", slow_car_ahead_counter);
        // publish obstacle if counter is reached
        if (slow_car_ahead_counter > NUM_SLOW_CAR_PUB
            // if not published obstacle recently
            && (ros::Time::now() - obstacle_last_published > ros::Duration(3.0)) 
            && (!slow_car_ahead_published || ros::Time::now() - slow_car_last_published > ros::Duration(10.0)) 
            && deleted_points - slow_car_last_published_deleted_points > 100
            // if distance to intersection is higher than MIN_DISTANCE_INTERSECTION
            && getDistanceToIntersection() > MIN_DISTANCE_INTERSECTION
        ) {
            
            ROS_INFO("publishing obstacle ahead");
            slow_car_ahead_published = true;
            slow_car_last_published = ros::Time::now();
            slow_car_last_published_deleted_points = deleted_points;
            obstacle_last_published = ros::Time::now();

            // raytrace in set area -> detect obstacles in area
            std::vector<RaytraceCollisionData> collisions = {};
            costmap_raytracer.raytraceSemiCircle(OBSTACLE_AREA, OBSTACLE_AREA_RADIUS, collisions);

            std::vector<geometry_msgs::Point> points = {};
            // map coordinates to obstacles
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
            msg.obstacles = points;

            obstacle_pub.publish(msg);
        }
        // consider obstacle as lost if decreasing slow car counter publish empty obstacle list
        if (slow_car_ahead_counter < NUM_SLOW_CAR_DEL
            && slow_car_ahead_published
            && (ros::Time::now() - obstacle_last_published > ros::Duration(3.0)) 
        ) {
            ROS_INFO("publishing loss of obstacle");
            slow_car_ahead_published = false;

            obstacle_last_published = ros::Time::now();
            psaf_messages::Obstacle msg;
            msg.id = obstacle_msg_id_counter++;
            msg.obstacles = {};

            obstacle_pub.publish(msg);
        }
    }

    /**
     * Checks the distance which is free on the global plan using the costmap
     *
     * @return true if a obstacle was found in the bounds of the costmap
     */
    bool PsafLocalPlanner::checkDistanceForward(double& distance, double &relative_x, double &relative_y)
    {
        tf2::Vector3 last_point, current_point, actual_point;
        tf2::convert(current_pose.pose.position, last_point);
        // on elevations the car position is != 0 unlike the map position
        last_point.setZ(0);
        actual_point = last_point;

        double sum_distance = 0;
        int count_error = 0;

        auto costmap = costmap_ros->getCostmap();
        auto bound_x = costmap->getSizeInCellsX();
        auto bound_y = costmap->getSizeInCellsY();

        // iterate over global plan up until {check_collision_max_distance} meters
        for (auto it = global_plan.begin(); it != global_plan.end(); ++it)
        {
            if (sum_distance > check_collision_max_distance)
                break;

            const geometry_msgs::PoseStamped &w = *it;
            tf2::convert(w.pose.position, current_point);
            sum_distance += tf2::tf2Distance(last_point, current_point);

            unsigned int cx, cy;
            // checks the costmap at the given position of the route; ignore if out of bounds of the local map
            if (costmap_ros->getCostmap()->worldToMap(current_point.getX(), current_point.getY(), cx, cy))
            {
                bool has_coll = false;
                // We need to check more than a single small square on the costmap
                // Checking only a small area we are sometimes unable to reliably detect small vehicles 
                // like bicycles which might not be completely centered on the route
                // Checking in a 3x3 square around the route position should be sufficient.
                // Has to be adapted for the resolultion of the local costmap
                for (int ix = -1; ix <= 1 && !has_coll; ix++) {
                    for (int iy = -1; iy <= 1 && !has_coll; iy++) {
                        if (cx <= 0 || cy <= 0 || cx + ix > bound_x || cy + iy > bound_y)
                            continue;

                        unsigned char cost = costmap_ros->getCostmap()->getCost(cx + ix, cy + iy);
                        // We are ignoring unknown costs in this case as we are clearing the costmap every iteration 
                        // with the {{psaf_obstacle_layer}} Packge
                        if (cost > COSTMAP_THRESHOLD && cost != costmap_2d::NO_INFORMATION) {
                            has_coll = true;
                        }
                    }
                }

                if (has_coll)
                {
                    count_error += 1;
                    if (count_error >= 2)
                    {
                        relative_x = current_point.getX() - actual_point.getX();
                        relative_y = current_point.getY() - actual_point.getY();
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