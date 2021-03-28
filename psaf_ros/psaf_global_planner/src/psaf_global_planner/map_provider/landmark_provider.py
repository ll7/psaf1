# !/usr/bin/env python

import glob
import os
import sys
from geometry_msgs.msg import Point
import rospy

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


class LandMarkPoint:

    def __init__(self, x: float, y: float, orientation: float, id: int):
        self.x = x
        self.y = y
        # get orientation without multiple rotations
        self.orientation = orientation % 360
        # get the positive angle
        if self.orientation < 0:
            self.orientation = self.orientation + 360

        self.mark_id = id

    def pos_as_point(self) -> Point:
        """
        Returns the position of the Landmark point as a Point object
        :return: Point object
        """
        return Point(self.x, self.y, 0)

    def __eq__(self, other):
        return self.mark_id == other.mark_id

    def __hash__(self):
        return hash(self.mark_id)


class LandMarkProvider:
    def __init__(self):
        self.landmarks = self._get_markings()

    def available_categories(self):
        """
        All available categories of of the stored landmarks
        :return: list of categories
        """
        return list(self.landmarks.keys())

    def get_marks_by_category(self, category: str):
        """
        Get all Landmarks of the given category
        :param category: the given category
        :return: landmarks
        """
        if category in self.landmarks.keys():
            return self.landmarks[category]
        else:
            return None

    def _get_markings(self):
        """
        Get all Carla Landmarks
        :return: dict of landmarks
        """
        try:
            # First of all, we need to create the client that will send the requests
            # to the simulator. Here we'll assume the simulator is accepting

            host = rospy.get_param('/carla/host', 'localhost')
            port = rospy.get_param('/carla/port', 2000)

            client = carla.Client(host, port)
            client.set_timeout(2.0)

            # Once we have a client we can retrieve the world that is currently
            # running.
            world = client.get_world()
            markings = {}
            marks = world.get_map().get_all_landmarks()
            for mark in marks:
                if mark.name in markings:
                    markings[mark.name].add(
                        LandMarkPoint(mark.transform.location.x, -mark.transform.location.y,
                                      mark.transform.rotation.yaw,
                                      int(mark.id)))
                else:
                    markings[mark.name] = set()
                    markings[mark.name].add(
                        LandMarkPoint(mark.transform.location.x, -mark.transform.location.y,
                                      mark.transform.rotation.yaw,
                                      int(mark.id)))
            return markings
        finally:
            pass
