
#include "CameraFusionService.h"
#include <set>
#include "boost/range/adaptors.hpp"
#include "boost/range/algorithm/copy.hpp"
#include "HelperAlgorithms.h"

namespace psaf_abstraction_layer {
   CameraFusionService::CameraFusionService(std::string role_name, std::string camera_group_name , double time_threshold) {

        // Init vars
        this->time_threshold_ns = time_threshold * 1e9;

        ros::NodeHandle private_nh("~/" + role_name + "/" + camera_group_name);
        // Publisher
        this->publisher = private_nh.advertise<psaf_messages::CombinedCameraImage>(
                "/psaf/sensors/" + role_name + "/fusionCamera/" + camera_group_name + "/fusion_image", 1);

        // Subscribers
        this->segmentation_subscriber = private_nh.subscribe(
                "/carla/" + role_name + "/camera/semantic_segmentation/" + camera_group_name + "/image_segmentation", 10,
                &CameraFusionService::onSegmentationImageCallback, this);
        this->rgb_subscriber = private_nh.subscribe("/carla/" + role_name + "/camera/rgb/" + camera_group_name + "/image_color", 10,
                                                    &CameraFusionService::onRGBImageCallback, this);
        this->depth_subscriber = private_nh.subscribe("/carla/" + role_name + "/camera/depth/" + camera_group_name + "/image_depth",
                                                      10,
                                                      &CameraFusionService::onDepthImageCallback, this);

    }


    void CameraFusionService::onSegmentationImageCallback(const sensor_msgs::Image &msg) {
        // Acquire lock
        const std::lock_guard<std::mutex> lock(this->processing_lock);
        this->segmentation_images[msg.header.stamp.toNSec()] = msg;
    }

    void CameraFusionService::onRGBImageCallback(const sensor_msgs::Image &msg) {
        // Acquire lock
        const std::lock_guard<std::mutex> lock(this->processing_lock);
        this->rgb_images[msg.header.stamp.toNSec()] = msg;
    }

    void CameraFusionService::onDepthImageCallback(const sensor_msgs::Image &msg) {
        // Acquire lock
        const std::lock_guard<std::mutex> lock(this->processing_lock);
        this->depth_images[msg.header.stamp.toNSec()] = msg;

    }

    void CameraFusionService::matchImages() {
        // Acquire lock
        const std::lock_guard<std::mutex> lock(this->processing_lock);
        // Get the time list and use a copy of them to capture only the current situation that might change during
        // the execution
        std::vector<uint64_t> segmentation_times;
        std::vector<uint64_t> rgb_times;
        std::vector<uint64_t> depth_times;
        // Extract keys using boost
        boost::copy(this->segmentation_images | boost::adaptors::map_keys,
                    std::back_inserter(segmentation_times));
        boost::copy(this->rgb_images | boost::adaptors::map_keys,
                    std::back_inserter(rgb_times));
        boost::copy(this->depth_images | boost::adaptors::map_keys,
                    std::back_inserter(depth_times));

        // find best matches always referenced to the segmentation list
        std::set<uint64_t> rgb_matches_ref;
        std::map<uint64_t, uint64_t> rgb_matches_map;
        diffBasedMatching(&segmentation_times, &rgb_times, &rgb_matches_ref, &rgb_matches_map, this->time_threshold_ns);
        std::set<uint64_t> depth_matches_ref;
        std::map<uint64_t, uint64_t> depth_matches_map;
        diffBasedMatching(&segmentation_times, &depth_times, &depth_matches_ref, &depth_matches_map, time_threshold_ns);


        //stop here if we couldn't find any matches that meet our appropriate conditions and wishes
        if (rgb_matches_ref.empty() || depth_matches_ref.empty()) {
            return;
        }
        // Combine both match list to find the matches that are matched with the same segmentation image
        // step 1: get the intersection of the key sets
        std::set<uint64_t> refs_intersection = intersection_of(rgb_matches_ref, depth_matches_ref);
        //stop here if we couldn't find any matches that meet our appropriate conditions and wishes
        if (refs_intersection.empty()) {
            return;
        }
        // step 2: Send all data
        uint64_t newest_time_match;
        for (uint64_t ref:refs_intersection) {
            newest_time_match = ref;
            //check if key sets contains ref of intersection
            if (rgb_matches_ref.find(ref) == rgb_matches_ref.end()) {
                continue;
            }
            if (depth_matches_ref.find(ref) == depth_matches_ref.end()) {
                continue;
            }
            try {
                psaf_messages::CombinedCameraImage msg;
                msg.segmentation = this->segmentation_images.at(ref);
                msg.rgb = this->rgb_images.at(rgb_matches_map.at(ref));
                msg.depth = this->depth_images.at(depth_matches_map.at(ref));
                this->publisher.publish(msg);
            } catch (std::out_of_range ex_out) {
                // We may expect this error so we ignore  it for shorter code
            }
        }
        // step 3: remove all images that are older than the newest key
        for (auto it = this->segmentation_images.begin(); it != this->segmentation_images.end();) {
            if (it->first <= newest_time_match) {
                this->segmentation_images.erase(it++);
            } else {
                (++it);
            }
        }
        uint64_t time_border = rgb_matches_map.at(newest_time_match);
        for (auto it = this->rgb_images.begin(); it != this->rgb_images.end();) {
            if (it->first <= time_border) {
                this->rgb_images.erase(it++);
            } else {
                (++it);
            }
        }
        time_border = depth_matches_map.at(newest_time_match);
        for (auto it = this->depth_images.begin(); it != this->depth_images.end();) {
            if (it->first <= time_border) {
                this->depth_images.erase(it++);
            } else {
                (++it);
            }
        }

    }

    void CameraFusionService::work() {
        while(!ros::isShuttingDown()) {
            this->matchImages();
        }
    }

    void CameraFusionService::start() {
       this->worker_thread = new std::thread(&CameraFusionService::work, this);

    }

    CameraFusionService::~CameraFusionService() {
        this->worker_thread->detach();
    }

}

// Begin main

int main(int argc, char **argv) {
    /**
      * The ros::init() function needs to see argc and argv so that it can perform
      * any ROS arguments and name remapping that were provided at the command line.
      * For programmatic remappings you can use a different version of init() which takes
      * remappings directly, but for most command-line programs, passing argc and argv is
      * the easiest way to do it.  The third argument to init() is the name of the node.
      *
      * You must call one of the versions of ros::init() before using any other
      * part of the ROS system.
      */
    ros::init(argc, argv, "sensor_fusion_service");
    ros::NodeHandle nh("~");
    std::string role_name;
    nh.getParam("role_name",role_name);
    std::string camera_group;
    nh.getParam("camera_group",camera_group);

    double time_threshold;
    nh.getParam("threshold_diff",time_threshold);


    using namespace psaf_abstraction_layer;
    CameraFusionService *service = new CameraFusionService(role_name, camera_group, time_threshold);
    service->start();
    /**
       * ros::spin() will enter a loop, pumping callbacks.  With this version, all
       * callbacks will be called from within this thread (the main one).  ros::spin()
       * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
       */
    ros::spin();

    return 0;
}
