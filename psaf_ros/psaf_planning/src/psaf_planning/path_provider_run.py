#!/usr/bin/env python

from psaf_planning.global_planner.path_provider_lanelet2 import PathProviderLanelet2
from psaf_planning.global_planner.path_provider_abstract import PathProviderAbstract
import rospy


def main():
    provider: PathProviderAbstract = PathProviderLanelet2(init_rospy=True)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
