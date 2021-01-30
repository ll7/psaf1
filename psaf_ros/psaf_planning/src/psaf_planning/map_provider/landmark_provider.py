# !/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


class LandMarkPoint:
    """
    Class that represents a LandMark Object and contains its absolute position, defined by x, y and its orientation,
    and its designated id.
    """

    def __init__(self, x: float, y: float, orientation: float, id: int):
        """
        LandMarkPointObject
        :param x: x position [m]
        :param y: y position [m]
        :param orientation: for this specific purpose: yaw_angle as a rotation by the z-axis [degree]
        :param id: landmark id, set by carla
        """
        self.x = x
        self.y = y
        # get orientation without multiple rotations
        self.orientation = orientation % 360
        # get the positive angle
        if self.orientation < 0:
            self.orientation = self.orientation + 360

        self.mark_id = id

    def __eq__(self, other):
        return self.mark_id == other.mark_id

    def __hash__(self):
        return hash(self.mark_id)


class LandMarkProvider:
    def __init__(self):
        self.landmarks = self._get_markings()

    def available_categories(self):
        return list(self.landmarks.keys())

    def get_marks_by_categorie(self, categorie: str):
        if categorie in self.landmarks.keys():
            return self.landmarks[categorie]
        else:
            return None

    def _get_markings(self):
        try:
            # First of all, we need to create the client that will send the requests
            # to the simulator. Here we'll assume the simulator is accepting
            # requests in the localhost at port 2000.
            client = carla.Client('localhost', 2000)
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
