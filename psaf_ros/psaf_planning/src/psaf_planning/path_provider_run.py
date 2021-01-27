#!/usr/bin/env python

from psaf_planning.global_planner.path_provider_lanelet2 import PathProviderLanelet2
from psaf_planning.global_planner.path_provider_abstract import PathProviderAbstract
from psaf_planning.global_planner.path_provider_common_roads import PathProviderCommonRoads
from psaf_planning.path_supervisor_common_roads import PathSupervisorCommonRoads
import rospy


def main():
    provider: PathProviderAbstract = PathSupervisorCommonRoads(init_rospy=True, enable_debug=False)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass