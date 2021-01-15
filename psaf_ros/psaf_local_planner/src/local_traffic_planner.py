import rospy
from std_msgs.msg import UInt8
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from psaf_messages.msg import TrafficSignInfo, SpeedSign, StopMark, TrafficLight
import numpy as np

class Local_Traffic_Planner():
    # TODO doc

    def __init__(self):
        self.velocity_publisher = rospy.Publisher("psaf_velocity_plan", UInt8, queue_size=1)

        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, self.callback_ego_vehicle_cmd)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, self.callback_ego_vehicle_cmd)

        self.acceptance_area = np.zeros([3,3])

    def callback_ego_vehicle_cmd(self,data: CarlaEgoVehicleControl):

        width = 0.34
        x1 = data.steer * (0.3 if data.steer > 0 else 0.6) + 0.5

        # Calculate coordinates
        x2 = min([x1 + width, 1])

        self.acceptance_area = np.ndarray([x1,0.2,x2,0.8])

    def callback_traffic_signs(self,traffic_signs:TrafficSignInfo):
        speed_signs = traffic_signs.speedSigns

        edited_speed_list = sorted(filter(lambda x: x.distance < 15,speed_signs),key=lambda x:x.distance)

        area = self.acceptance_area
        matches = filter(lambda sign: area[0] <= sign.x.data <= area[2] and area[1] <= sign.y.data <= area[3],edited_speed_list)

        best = next(iter(matches), None)

        msg = UInt8(best.limit)

        self.velocity.publish(msg)
        print(f"I updated the velocity to {msg}")


if __name__ == '__main__':
    planner = Local_Traffic_Planner()


