

#ifndef PSAF_ABSTRACTION_LAYER_CAMERAFUSIONSERVICE_H
#define PSAF_ABSTRACTION_LAYER_CAMERAFUSIONSERVICE_H

// General includes
#include <string>
#include <vector>
#include <thread>
// Ros includes
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// internal includes
#include "psaf_messages/CombinedCameraImage.h"

namespace psaf_abstraction_layer {
    /**
     * The service class that takes the three streams of segmentation, rgb and depth camera and create a new topic
     * with the fusion camera data of all three camera
     */
    class CameraFusionService {

    public:
        /**
         * The constructor of the service
         * @param role_name the role name of the vehicle
         * @param camera_group_name the camera group name e.g. "front"
         * @param time_threshold the threshold for matching the images (= max time diff between every matched image)
         */
        CameraFusionService(std::string role_name, std::string camera_group_name, double time_threshold = .1);

        /**
         * The destructor
         */
        ~CameraFusionService();

        /**
         * Starts the service in background using a thread
         */
        void start();

    private:

        /**
         * Storage for the threshold
         */
        uint64_t time_threshold_ns;

        /**
         * result publisher
         */
        ros::Publisher publisher;

        /**
         * The segmentation image subscriber
         */
        ros::Subscriber segmentation_subscriber;
        /**
        * The rgb image subscriber
        */
        ros::Subscriber rgb_subscriber;
        /**
        * The depth image subscriber
        */
        ros::Subscriber depth_subscriber;

        /**
         * map with time in ns  -> image msg for segmentation camera
         */
        std::map<uint64_t, sensor_msgs::Image> segmentation_images;
        /**
         * map with time in ns  -> image msg for rgb camera
         */
        std::map<uint64_t, sensor_msgs::Image> rgb_images;
        /**
         * map with time in ns  -> image msg for depth camera
         */
        std::map<uint64_t, sensor_msgs::Image> depth_images;

        /**
         * Concurrency lock
         */
        std::mutex processing_lock;
        /**
         * The worker thread
         */
        std::thread *worker_thread;

        /**
         * Callback for segmentation image subscriber
         * @param msg the image message
         */
        void onSegmentationImageCallback(const sensor_msgs::Image &msg);

        /**
                * Callback for rgb image subscriber
                * @param msg the image message
                */
        void onRGBImageCallback(const sensor_msgs::Image &msg);

        /**
                * Callback for depth image subscriber
                * @param msg the image message
                */
        void onDepthImageCallback(const sensor_msgs::Image &msg);

        /**
         * Matches the images stored in the map and publishes the results
         */
        void matchImages();

        /**
         * Looping method called inside the worker thread
         */
        void work();

    };

}


#endif //PSAF_ABSTRACTION_LAYER_CAMERAFUSIONSERVICE_H
