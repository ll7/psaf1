//
// Created by psaf1 on 08.02.21.
//

#include "SensorFusionService.h"
#include <set>
#include "boost/range/adaptors.hpp"
#include "boost/range/algorithm/copy.hpp"
#include <unordered_set>
namespace psaf_abstraction_layer {
   SensorFusionService::SensorFusionService(std::string role_name, double time_threshold) {

        // Init vars
        this->time_threshold_ns = time_threshold * 1e9;

        ros::NodeHandle private_nh("~/" + role_name + "/" + "front");
        // Publisher
        this->publisher = private_nh.advertise<psaf_messages::CombinedCameraImage>(
                "/psaf/sensors/" + role_name + "/fusionCamera", 1);

        // Subscribers
        const std::string id = "front";
        this->segmentation_subscriber = private_nh.subscribe(
                "/carla/" + role_name + "/camera/semantic_segmentation/" + id + "/image_segmentation", 10,
                &SensorFusionService::onSegmentationImageCallback, this);
        this->rgb_subscriber = private_nh.subscribe("/carla/" + role_name + "/camera/rgb/" + id + "/image_color", 10,
                                                    &SensorFusionService::onRGBImageCallback, this);
        this->depth_subscriber = private_nh.subscribe("/carla/" + role_name + "/camera/depth/" + id + "/image_depth",
                                                      10,
                                                      &SensorFusionService::onDepthImageCallback, this);

    }


    void SensorFusionService::onSegmentationImageCallback(const sensor_msgs::Image &msg) {
        // Acquire lock
        const std::lock_guard<std::mutex> lock(this->processing_lock);
        this->segmentation_images[msg.header.stamp.toNSec()] = msg;
    }

    void SensorFusionService::onRGBImageCallback(const sensor_msgs::Image &msg) {
        // Acquire lock
        const std::lock_guard<std::mutex> lock(this->processing_lock);
        this->rgb_images[msg.header.stamp.toNSec()] = msg;
    }

    void SensorFusionService::onDepthImageCallback(const sensor_msgs::Image &msg) {
        // Acquire lock
        const std::lock_guard<std::mutex> lock(this->processing_lock);
        this->depth_images[msg.header.stamp.toNSec()] = msg;

    }

    void SensorFusionService::matchImages() {
        // Acquire lock
        const std::lock_guard<std::mutex> lock(this->processing_lock);
        // Get the time list and use a copy of them to capture only the current situation that might change during
        // the execution
        std::vector<timeSimple_t> segmentation_times;
        std::vector<timeSimple_t> rgb_times;
        std::vector<timeSimple_t> depth_times;
        // Extract keys using boost
        boost::copy(this->segmentation_images | boost::adaptors::map_keys,
                    std::back_inserter(segmentation_times));
        boost::copy(this->rgb_images | boost::adaptors::map_keys,
                    std::back_inserter(rgb_times));
        boost::copy(this->depth_images | boost::adaptors::map_keys,
                    std::back_inserter(depth_times));

        // find best matches always referenced to the segmentation list
        std::set<timeSimple_t> rgb_matches_ref;
        std::map<timeSimple_t, timeSimple_t> rgb_matches_map;
        diffBasedMatching(&segmentation_times, &rgb_times, &rgb_matches_ref, &rgb_matches_map, this->time_threshold_ns);
        std::set<timeSimple_t> depth_matches_ref;
        std::map<timeSimple_t, timeSimple_t> depth_matches_map;
        diffBasedMatching(&segmentation_times, &depth_times, &depth_matches_ref, &depth_matches_map, time_threshold_ns);


        //stop here if we couldn't find any matches that meet our appropriate conditions and wishes
        if (rgb_matches_ref.empty() || depth_matches_ref.empty()) {
            return;
        }
        // Combine both match list to find the matches that are matched with the same segmentation image
        // step 1: get the intersection of the key sets
        std::set<timeSimple_t> refs_intersection = intersection_of(rgb_matches_ref, depth_matches_ref);
        //stop here if we couldn't find any matches that meet our appropriate conditions and wishes
        if (refs_intersection.empty()) {
            return;
        }
        // step 2: Send all data
        timeSimple_t newest_time_match;
        for (timeSimple_t ref:refs_intersection) {
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
        timeSimple_t time_border = rgb_matches_map.at(newest_time_match);
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

    void SensorFusionService::work() {
        while(!ros::isShuttingDown()) {
            this->matchImages();
        }
    }

    void SensorFusionService::start() {
       this->worker_thread = new std::thread(&SensorFusionService::work,this);

    }


    void diffBasedMatching(std::vector<timeSimple_t> *list_a, std::vector<timeSimple_t> *list_b,
                                              std::set<timeSimple_t> *results_set_a,
                                              std::map<timeSimple_t, timeSimple_t> *result_map_a_to_b,
                                              timeSimple_t threshold) {

        std::vector<timeSimple_t> *bigger_sorted;
        std::vector<timeSimple_t> *smaller_sorted;
        bool swapped = false;
        if (list_a->size() <= list_b->size()) {
            smaller_sorted = list_a;
            bigger_sorted = list_b;
            swapped = false;
        } else {
            bigger_sorted = list_a;
            smaller_sorted = list_b;
            swapped = true;
        }

        int lower_bound = 0;
        int upper_bound = bigger_sorted->size();

        for (timeSimple_t reference_value : *smaller_sorted) {
            // The last value of the bigger_sorted_list that might match to the current lessFreqValue

            timeSimple_t possible_match;
            bool have_match = false;
            // counter for the checked values in ine moreDataSorted list
            int counter = 0;

            for (int i = lower_bound; i < upper_bound; ++i) {
                // The value of the list that contains more elements -> higher measuring frequency
                timeSimple_t more_freq_value = bigger_sorted->at(i);
                if (have_match && std::abs((long) (reference_value - possible_match)) <
                                  std::abs((long) (reference_value - more_freq_value))) {
                    // If there is already possible match and if the difference between the current timestamp
                    // and the reference timestamp is greater than difference between timestamp of the last
                    // possible match and the  reference timestamp, all upcoming elements will have a greater
                    // difference and won't be an appropriate match
                    break;
                }
                // Store possible match
                possible_match = more_freq_value;
                have_match = true;
                // Increment counter
                counter++;
                if (reference_value <= possible_match) {
                    // if the reference time is older than the value of possible match all new values of the
                    // sorted(!) list will have a bigger difference
                    break;
                }
                if (have_match) {
                    // if the match full fills the condition that the diff is smaller than the threshold add iut to the list
                    if (std::abs((long) (reference_value - possible_match) < threshold)) {
                        if (!swapped) {
                            results_set_a->insert(reference_value);
                            result_map_a_to_b->insert(
                                    std::pair<timeSimple_t,timeSimple_t>(reference_value,possible_match));
                        } else {
                            results_set_a->insert(possible_match);
                            result_map_a_to_b->insert(
                                    std::pair<timeSimple_t,timeSimple_t>(possible_match,reference_value));
                        }
                    }
                    //increase the lower bound because all value with smaller timestamp that the match won't be suitable
                    // for future matches
                    lower_bound += counter;
                }


            }

        }


    }

    template<typename T>
    std::set<T> intersection_of(const std::set<T> &a, const std::set<T> &b) {
        std::set<T> rtn;
        std::unordered_multiset<T> st;
        std::for_each(a.begin(), a.end(), [&st](const T &k) { st.insert(k); });
        std::for_each(b.begin(), b.end(),
                      [&st, &rtn](const T &k) {
                          auto iter = st.find(k);
                          if (iter != st.end()) {
                              rtn.insert(k);
                              st.erase(iter);
                          }
                      }
        );
        return rtn;
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
    ros::init(argc, argv, "listener");

    using namespace psaf_abstraction_layer;
    SensorFusionService *service = new SensorFusionService("ego_vehicle",0.1);
    service->start();
    /**
       * ros::spin() will enter a loop, pumping callbacks.  With this version, all
       * callbacks will be called from within this thread (the main one).  ros::spin()
       * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
       */
    ros::spin();

    return 0;
}
