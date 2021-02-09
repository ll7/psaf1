//
// Created by psaf1 on 08.02.21.
//

#ifndef PSAF_ABSTRACTION_LAYER_SENSORFUSIONSERVICE_H
#define PSAF_ABSTRACTION_LAYER_SENSORFUSIONSERVICE_H

// General includes
#include <string>
#include <vector>
#include <thread>
// Ros includes
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// internal includes
#include "psaf_messages/CombinedCameraImage.h"

// Aliases
using timeSimple_t = uint64_t;

namespace psaf_abstraction_layer {
    class SensorFusionService {

    public:
        SensorFusionService(std::string role_name,double time_threshold=.1);

        ~SensorFusionService();

        void start();

    private:

        timeSimple_t time_threshold_ns;

        ros::Publisher publisher;

        ros::Subscriber segmentation_subscriber;
        ros::Subscriber rgb_subscriber;
        ros::Subscriber depth_subscriber;

        std::map<timeSimple_t,sensor_msgs::Image> segmentation_images;
        std::map<timeSimple_t,sensor_msgs::Image> rgb_images;
        std::map<timeSimple_t,sensor_msgs::Image> depth_images;

        std::mutex processing_lock;
        std::thread *worker_thread;

        void onSegmentationImageCallback(const sensor_msgs::Image &msg);
        void onRGBImageCallback(const sensor_msgs::Image &msg);
        void onDepthImageCallback(const sensor_msgs::Image &msg);

        void matchImages();

        void work();

    };

    void
    diffBasedMatching(std::vector<timeSimple_t> *list_a, std::vector<timeSimple_t> *list_b,
                      std::set<timeSimple_t> *results_set_a,
                      std::map<timeSimple_t, timeSimple_t> *result_map_a_to_b, timeSimple_t threshold);
    template<typename T>
    std::set<T> intersection_of(const std::set<T> &a, const std::set<T> &b);
}


#endif //PSAF_ABSTRACTION_LAYER_SENSORFUSIONSERVICE_H
