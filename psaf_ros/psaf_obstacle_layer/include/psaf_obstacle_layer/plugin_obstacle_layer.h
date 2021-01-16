#pragma once
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/obstacle_layer.h>


namespace psaf_obstacle_layer {
    class PsafObstacleLayer : public costmap_2d::ObstacleLayer {
        void updateCosts (costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override;
    };
};