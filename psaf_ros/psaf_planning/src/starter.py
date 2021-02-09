#!/usr/bin/env python
import rospy
from psaf_messages.msg import PlanningInstruction
from sensor_msgs.msg import NavSatFix

def main():
    rospy.init_node('starter', anonymous=True)
    plan_pub = rospy.Publisher("/psaf/goal/set_instruction", PlanningInstruction, queue_size=10)
    sat = NavSatFix(latitude=0, longitude=0, altitude=0)
    msg = PlanningInstruction(goalPoint=sat, obstacleDistanceLeft=100, obstacleDistanceForward=100, planUTurn=True)
    while plan_pub.get_num_connections() == 0:
        rospy.sleep(1)
        print("wait")
    plan_pub.publish(msg)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
