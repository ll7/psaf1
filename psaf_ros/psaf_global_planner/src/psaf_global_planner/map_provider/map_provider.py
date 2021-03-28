#!/usr/bin/env python

import rospy
import os

from io import BytesIO
from lxml import etree
from carla_msgs.msg import CarlaWorldInfo
from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2lanelet.network import Network
from opendrive2lanelet.osm.lanelet2osm import L2OSMConverter
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Tag
from commonroad.scenario.scenario import Scenario, ScenarioID

import tempfile


class MapProvider:

    def __init__(self, init_rospy: bool = False):
        self.osm_file = tempfile.NamedTemporaryFile(suffix=".osm")
        # opendrive map received from the _world_info_subscriber
        self.map: str = None
        # name of the map received from the _world_info_subscriber
        self.map_name: str = None
        # boolean that represents the availability of the opendrive map -> true if the map was received by the subscriber
        self.map_ready: bool = False
        # starts the logging node , normally only needed if the module is used independently
        if init_rospy:
            rospy.init_node('mapProvider', anonymous=True)

        # Subscriber to receive the currently loaded map
        self._world_info_subscriber = rospy.Subscriber(
            "/carla/world_info", CarlaWorldInfo, self.update_world)

    def __del__(self):
        # closing the TemporaryFile deletes it
        self.osm_file.close()

    def update_world(self, world_info: CarlaWorldInfo):
        """
        Check if a new map was sent and receive it
        :param world_info: Carla world info data
        """
        self.map_ready = False
        rospy.loginfo("MapProvider: Received new map info")
        if self.map_name == world_info.map_name:
            rospy.loginfo("MapProvider: Map already loaded")
        else:
            self.map_name = world_info.map_name
            self.map = world_info.opendrive
            self.map_ready = True
            rospy.loginfo("MapProvider: Received: " + self.map_name)

    def convert_to_osm(self) -> str:
        """
        Create a temporary OpenStreetMap file that can be used to generate a lanelet2 LaneletMap
        """
        lanelet = self.convert_od_to_lanelet()
        if lanelet is not None:
            l2osm = L2OSMConverter(
                "+proj=omerc +lat_0=0 +lonc=0 +alpha=0 +k=1 +x_0=0 +y_0=0 +gamma=0 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0")
            openstreetmap = etree.tostring(l2osm(lanelet), xml_declaration=True, encoding="UTF-8", pretty_print=True)
            self.osm_file.write(openstreetmap)
            rospy.loginfo("MapProvider: Temporary file created:" + os.path.abspath(self.osm_file.name))
            return os.path.abspath(self.osm_file.name)
        else:
            rospy.logerr("MapProvider: lanelet not available")
            rospy.logerr("MapProvider: Couldn't create temporary file")
            return ""

    """
    Currently not in use since CommonRoad lanelets are not compatible with lanelet2
    """

    def convert_od_to_lanelet(self) -> Scenario:
        """
        Create a CommonRoad scenario from the OpenDrive received OpenDrive map
        :return: Scenario if the map is ready, None instead
        """
        lanelet: Scenario = None
        if self.map_ready:
            rospy.loginfo("MapProvider: Start conversion...")
            opendrive = parse_opendrive(etree.parse(BytesIO(self.map.encode('utf-8'))).getroot())
            roadNetwork = Network()
            roadNetwork.load_opendrive(opendrive)
            scenario_id = ScenarioID(country_id="DEU", map_name="psaf")
            lanelet = roadNetwork.export_commonroad_scenario(benchmark_id=scenario_id)
            rospy.loginfo("MapProvider: Conversion done!")
        return lanelet

    """
    The generate methods are just for debugging purposes 
    """

    def generate_osm_file(self):
        lanelet = self.convert_od_to_lanelet()
        if lanelet is not None:
            l2osm = L2OSMConverter(
                "+proj=omerc +lat_0=0 +lonc=0 +alpha=0 +k=1 +x_0=0 +y_0=0 +gamma=0 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0")
            openstreetmap = etree.tostring(l2osm(lanelet), xml_declaration=True, encoding="UTF-8", pretty_print=True)

            # write osm file
            with open(self.map_name + ".osm", "wb") as file_out:
                file_out.write(openstreetmap)
            rospy.loginfo("MapProvider: Wrote file" + self.map_name + ".osm")
        else:
            rospy.logerr("MapProvider: lanelet not available")
            rospy.logerr("MapProvider: No file generated")

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
            rospy.loginfo("MapProvider: Wrote file" + self.map_name + ".xml")
        else:
            rospy.logerr("MapProvider: lanelet not available")
            rospy.logerr("MapProvider: No file generated")


def main():
    provider = MapProvider(True)
    while not provider.map_ready:
        rospy.loginfo("Waiting")
    provider.convert_od_to_lanelet()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
