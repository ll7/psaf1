//
// Created by psaf1 on 09.02.21.
//

#include "HelperAlgorithms.h"

#include "boost/range/adaptors.hpp"



namespace psaf_abstraction_layer {

    /**
     * Matches the given two list with
     * @param list_a the list that contains the reference values
     * @param list_b the other list that will be match with list a
     * @param results_set_a the set of keys
     * @param result_map_a_to_b the map that maps the values of the ref list to the matches out of list_b
     * @param threshold a threshold value of the maximal diff between the values of every list
     */
    void diffBasedMatching(std::vector <uint64_t> *list_a, std::vector <uint64_t> *list_b,
                           std::set <uint64_t> *results_set_a,
                           std::map <uint64_t, uint64_t> *result_map_a_to_b,
                           uint64_t threshold) {

        std::vector <uint64_t> *bigger_sorted;
        std::vector <uint64_t> *smaller_sorted;
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

        for (uint64_t reference_value : *smaller_sorted) {
            // The last value of the bigger_sorted_list that might match to the current lessFreqValue

            uint64_t possible_match;
            bool have_match = false;
            // counter for the checked values in ine moreDataSorted list
            int counter = 0;

            for (int i = lower_bound; i < upper_bound; ++i) {
                // The value of the list that contains more elements -> higher measuring frequency
                uint64_t more_freq_value = bigger_sorted->at(i);
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
                                    std::pair<uint64_t, uint64_t>(reference_value, possible_match));
                        } else {
                            results_set_a->insert(possible_match);
                            result_map_a_to_b->insert(
                                    std::pair<uint64_t, uint64_t>(possible_match, reference_value));
                        }
                    }
                    //increase the lower bound because all value with smaller timestamp that the match won't be suitable
                    // for future matches
                    lower_bound += counter;
                }


            }

        }

    

    }

}