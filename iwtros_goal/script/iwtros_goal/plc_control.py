#!/usr/bin/env python
import rospy
from iwtros_msgs.msg import plcControl 
from iwtros_msgs.msg import kukaControl

from time import sleep
import snap7 as sn
from snap7.util import *
import struct

readArea = 0x82     #area for Q memory
writeArea = 0x81    #area for I memory

plc = sn.client.Client()
plc.connect("192.168.0.1", 0, 1)
start = 200      # location we are going to start the read
length = 1     # length in bytes of the read

bit0 = 0        # get_controlMsgs.funtion: 1 << bit0
bit1 = 1
bit2 = 2 

responseFromKUKA = False
ReachedHomeKUKA = False

def callback(data):
    global responseFromKUKA, ReachedHomeKUKA
    responseFromKUKA = True
    rospy.sleep(1)  
    mByte = plc.read_area(readArea, 0, start, length)
    set_bool(mByte, 0, bit0, 0)
    set_bool(mByte, 0, bit1, 0)
    set_bool(mByte, 0, bit2, 0)
    if(data.ReachedHome):
        set_bool(mByte, 0, 0, 1)
        plc.write_area(writeArea, 0, start, mByte)
        rospy.loginfo("Reached Home")
        ReachedHomeKUKA = True
        rospy.sleep(0.1)
    if(data.ConveyorPlaced):
        set_bool(mByte, 0, bit2, 1)
        plc.write_area(writeArea, 0, start, mByte)
        rospy.loginfo("Placed on conveyor belt")
        rospy.sleep(0.1)
    if(data.DHBWPlaced):
        set_bool(mByte, 0, bit1, 1)
        plc.write_area(writeArea, 0, start, mByte)
        rospy.loginfo("Placed on DHBW belt")
        rospy.sleep(0.1)
    ReachedHomeKUKA = False
    

def controller():
    global responseFromKUKA, ReachedHomeKUKA
    rospy.init_node("plc_controller_node", anonymous=False)
    pub = rospy.Publisher('plc_control', plcControl, queue_size=10)
    sub = rospy.Subscriber('plc_listener', kukaControl, callback)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # default control messages
        controlMsgs = plcControl()
        controlMsgs.MoveHome = False
        controlMsgs.ConveyorPickPose = False
        controlMsgs.DHBWPickPose = False

        if(responseFromKUKA == False):
            rospy.sleep(1)               # Wait until calback is clear
            mByte = plc.read_area(readArea, 0, start, length)

            if(get_bool(mByte, 0, bit0) and (ReachedHomeKUKA == False)):
                controlMsgs.MoveHome = True
                rospy.loginfo("Moving IIWA to Home Pose")
            if(get_bool(mByte, 0, bit1)):
                controlMsgs.DHBWPickPose = True
                rospy.loginfo("Moving IIWA to DHBW Pick Pose")
                ReachedHomeKUKA = False
            if(get_bool(mByte, 0, bit2)):
                controlMsgs.ConveyorPickPose = True
                rospy.loginfo("Moving IIWA to Conveyor Pick Pose")
                ReachedHomeKUKA = False
            
            pub.publish(controlMsgs)

        rate.sleep()

        


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInternalException:
        plc.disconnect()
        pass