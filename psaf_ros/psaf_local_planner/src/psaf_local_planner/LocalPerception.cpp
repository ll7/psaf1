#include "psaf_local_planner/plugin_local_planner.h"


namespace psaf_local_planner
{

    void PsafLocalPlanner::checkForSlowCar(double velocity_distance_diff) {
        // Check if there should be a line change if speed is slower than x for y iterations
        if (velocity_distance_diff > 5) {
            slow_car_ahead_counter = std::min(50, slow_car_ahead_counter + 1);
        } else {
            slow_car_ahead_counter = std::max(0, slow_car_ahead_counter - 2);
        }

        ROS_INFO("slow car counter: %d", slow_car_ahead_counter);

        if (slow_car_ahead_counter > 30 
            && (ros::Time::now() - obstacle_last_published > ros::Duration(3.0)) 
            && (!slow_car_ahead_published || ros::Time::now() - slow_car_last_published > ros::Duration(10.0)) 
            && deleted_points - slow_car_last_published_deleted_points > 100
            && getDistanceToIntersection() > 30
        ) {
            
            ROS_INFO("publishing obstacle ahead");
            slow_car_ahead_published = true;
            slow_car_last_published = ros::Time::now();
            slow_car_last_published_deleted_points = deleted_points;


            std::vector<RaytraceCollisionData> collisions = {};
            costmap_raytracer.raytraceSemiCircle(M_PI * 3/2, 30, collisions);

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
            msg.obstacles = points;

            obstacle_pub.publish(msg);
        }

        if (slow_car_ahead_counter < 10 
            && slow_car_ahead_published
            && (ros::Time::now() - obstacle_last_published > ros::Duration(3.0)) 
        ) {
            ROS_INFO("publishing loss of obstacle");
            slow_car_ahead_published = false;

            psaf_messages::Obstacle msg;
            msg.id = obstacle_msg_id_counter++;
            msg.obstacles = {};

            obstacle_pub.publish(msg);
        }
    }

    /**
     * Returns false if it failed because something is close; true if out of bounds
     */
    bool PsafLocalPlanner::checkDistanceForward(double& distance, double &relative_x, double &relative_y)
    {
        tf2::Vector3 last_point, current_point, acutal_point;
        tf2::convert(current_pose.pose.position, last_point);
        last_point.setZ(0);
        acutal_point = last_point;

        double sum_distance = 0;
        int count_error = 0;

        auto costmap = costmap_ros->getCostmap();
        auto model = base_local_planner::CostmapModel(*costmap);
        auto footprint = costmap_ros->getRobotFootprint();
        auto bound_x = costmap->getSizeInCellsX();
        auto bound_y = costmap->getSizeInCellsY();

        ROS_INFO("footprint: %f %f", footprint[0].x, footprint[0].y);

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
                // TODO!!! DOES NOT WORK YET!!! 
                // current_point instead of current pose !!!!!!
                // int cost = model.footprintCost(w.pose.position, footprint, 2.0, 3.0);

                bool has_coll = false;

                for (int ix = -1; ix <= 1 && !has_coll; ix++) {
                    for (int iy = -1; iy <= 1 && !has_coll; iy++) {
                        if (cx <= 0 || cy <= 0 || cx + ix > bound_x || cy + iy > bound_y)
                            continue;

                        unsigned char cost = costmap_ros->getCostmap()->getCost(cx + ix, cy + iy);
                        if (cost > 128 && cost != costmap_2d::NO_INFORMATION) {
                            has_coll = true;
                        }
                    }
                }
                // unsigned char cost = costmap_ros->getCostmap()->getCost(cx, cy);

                if (has_coll)
                {
                    count_error += 1;
                    if (count_error >= 2)
                    {
                        ROS_WARN("cost at %f %f", current_point.getX() - acutal_point.getX(), current_point.getY() - acutal_point.getY());
                        relative_x = current_point.getX() - acutal_point.getX();
                        relative_y = current_point.getY() - acutal_point.getY();
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