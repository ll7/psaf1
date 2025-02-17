#include "psaf_obstacle_layer/plugin_obstacle_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(psaf_obstacle_layer::PsafObstacleLayer, costmap_2d::Layer)

namespace psaf_obstacle_layer
{
    
    void PsafObstacleLayer::updateCosts (costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
        static int counter = 0;

        // Updating first, then clearing works better than the other way around
        // Prevents flickering of the cost map
        costmap_2d::ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);

        costmap_2d::ObstacleLayer::reset();
    }
} // namespace psaf_local_planner

