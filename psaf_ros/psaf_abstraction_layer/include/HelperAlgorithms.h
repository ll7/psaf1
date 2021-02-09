//
// Created by psaf1 on 09.02.21.
//

#ifndef PSAF_ABSTRACTION_LAYER_HELPERALGORITHMS_H
#define PSAF_ABSTRACTION_LAYER_HELPERALGORITHMS_H
#include <set>
#include "ros/ros.h"

namespace psaf_abstraction_layer {

    void  diffBasedMatching(std::vector<uint64_t> *list_a, std::vector<uint64_t> *list_b,
                      std::set<uint64_t> *results_set_a,
                      std::map<uint64_t, uint64_t> *result_map_a_to_b, uint64_t threshold);
    template<typename T>
    std::set<T> intersection_of(const std::set<T> &a, const std::set<T> &b);
}

#endif //PSAF_ABSTRACTION_LAYER_HELPERALGORITHMS_H
