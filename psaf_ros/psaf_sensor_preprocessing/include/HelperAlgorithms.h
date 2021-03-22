
#ifndef PSAF_ABSTRACTION_LAYER_HELPERALGORITHMS_H
#define PSAF_ABSTRACTION_LAYER_HELPERALGORITHMS_H
#include <set>
#include <unordered_set>
#include "ros/ros.h"

namespace psaf_abstraction_layer {

    void  diffBasedMatching(std::vector<uint64_t> *list_a, std::vector<uint64_t> *list_b,
                      std::set<uint64_t> *results_set_a,
                      std::map<uint64_t, uint64_t> *result_map_a_to_b, uint64_t threshold);

    /**
     * Creates an intersection of two sets
     * @tparam T the generic type of the set
     * @param a first set
     * @param b second set
     * @return the intersection set
     */
    template<typename T>
    std::set <T> intersection_of(const std::set <T> &a, const std::set <T> &b) {
        std::set <T> rtn;
        std::unordered_multiset <T> st;
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
    };
}

#endif //PSAF_ABSTRACTION_LAYER_HELPERALGORITHMS_H
