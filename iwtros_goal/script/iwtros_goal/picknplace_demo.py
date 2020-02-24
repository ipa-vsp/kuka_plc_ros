#!/usr/bin/env python
import rospy
import math
from pilz_robot_programming import *
from geometry_msgs.msg import Pose, Point
import moveit_commander
import tf2_ros
import tf2_geometry_msgs


__REQUIRED_API_VERSION__ = "1" 
__ROBOT_VELOCITY__ = 0.1
__ROBOT_VELOCITY_LIN__ = 0.07
__ROBOT_ACCELERATION_LIN__ = 0.08
PLANNING_GROUP = "iiwa_arm"

def start_programme():
    rospy.loginfo("Program started")
    # Pick and Place Pose is above the oject
    pick_pose = Pose(position=Point(0,0,0), orientation=from_euler(math.radians(0), math.radians(0), math.radians(0)))
    place_pose = Pose(position=Point(0,0,0), orientation=from_euler(math.radians(0), math.radians(0), math.radians(0)))
    start_pose = Pose(position=Point(0.35,0,0.4), orientation=from_euler(math.radians(130), math.radians(0), math.radians(180)))

    iiwa.move(Ptp(goal=start_pose, 
                vel_scale=0.4, 
                acc_scale=0.45, 
                reference_frame="iiwa_link_0", 
                planning_group=PLANNING_GROUP, 
                target_link="iiwa_link_ee"))

    iiwa.move(Lin(goal=Pose(position=Point(0, 0, 0.1)), 
                    vel_scale=0.1, 
                    acc_scale=0.1, 
                    reference_frame="iiwa_link_ee", 
                    planning_group=PLANNING_GROUP, 
                    target_link="iiwa_link_ee"))
    
    iiwa.move(Lin(goal=Pose(position=Point(0, 0, -0.1)), 
                    vel_scale=0.1, 
                    acc_scale=0.1, 
                    reference_frame="iiwa_link_ee", 
                    planning_group=PLANNING_GROUP, 
                    target_link="iiwa_link_ee"))


if __name__ == "__main__":
    rospy.init_node("iiwa_move_node")
    iiwa = Robot(__REQUIRED_API_VERSION__)
    start_programme()
