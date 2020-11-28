#!/usr/bin/env python

import rospy

from io import BytesIO
from lxml import etree
from carla_msgs.msg import CarlaWorldInfo
from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2lanelet.network import Network
from opendrive2lanelet.osm.lanelet2osm import L2OSMConverter

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Tag
from commonroad.scenario.scenario import Scenario


class GPS_Position:
    """
    Position representation
    """
    def __init__(self, latitude: float = 0., longitude: float = 0., altitude: float = 0.):
        """
        Construct
        :param latitude: the latitude
        :param longitude: the longitude
        :param altitude: the altitude
        """
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def __str__(self) -> str:
        return "Pos long:'{}'-lat:'{}'-alt:'{}'".format(self.longitude,self.latitude,self.altitude)


class MapProvider:

    def __init__(self):
        self.map: str = None
        self.map_ready: bool = False
        self.map_name: str = None


        self._world_info_subscriber = rospy.Subscriber(
            "/carla/world_info", CarlaWorldInfo, self.update_world)

    def update_world(self, world_info: CarlaWorldInfo):
        """
        Check if a new map was sent and receive it
        """
        self.map_ready = False
        rospy.loginfo("Received new map info")
        if self.map_name == world_info.map_name:
            rospy.loginfo("Map already loaded")
        else:
            self.map_name = world_info.map_name
            self.map = world_info.opendrive
            self.map_ready = True
            rospy.loginfo("Received: " + self.map_name)

    def convert_od_to_lanelet(self) -> Scenario:
        """
        Creat a CommonRoad scenario from the OpenDrive received OpenDrive map
        """
        lanelet: Scenario = None
        if self.map_ready:
            rospy.loginfo("Start conversion...")
            opendrive = parse_opendrive(etree.parse(BytesIO(self.map.encode('utf-8'))).getroot())
            roadNetwork = Network()
            roadNetwork.load_opendrive(opendrive)
            lanelet = roadNetwork.export_commonroad_scenario()
            rospy.loginfo("Conversion done!")
        return lanelet

    def generate_osm_file(self):
        lanelet = self.convert_od_to_lanelet()
        if lanelet is not None:
            l2osm = L2OSMConverter(
                "+proj=omerc +lat_0=49 +lonc=8 +alpha=0 +k=1 +x_0=0 +y_0=0 +gamma=0 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0")
            openstreetmap = etree.tostring(l2osm(lanelet), xml_declaration=True, encoding="UTF-8", pretty_print=True)

            # write osm file
            with open(self.map_name + ".osm", "wb") as file_out:
                file_out.write(openstreetmap)
        else:
            rospy.logerr("lanelet not available")

    def generate_com_road_file(self):
        lanelet = self.convert_od_to_lanelet()
        if lanelet is not None:
            writer = CommonRoadFileWriter(
                scenario=lanelet,
                planning_problem_set=PlanningProblemSet(),
                author="Psaf1",
                affiliation="",
                source="MapProvider",
                tags={Tag.URBAN, Tag.HIGHWAY},
            )
            # write CommonRoad data to xml file
            writer.write_to_file(self.map_name + ".xml", OverwriteExistingFile.ALWAYS)
        else:
            rospy.logerr("lanelet not available")


def main():
    provider = MapProvider()
    while not provider.map_ready:
        rospy.loginfo("Waiting")
    provider.generate_osm_file()
    provider.generate_com_road_file()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass