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
plc2 = sn.client.Client()
plc.connect("192.168.0.1", 0, 1)
plc2.connect("192.168.0.1", 0, 1)
start = 200      # location we are going to start the read
length = 1     # length in bytes of the read

bit0 = 0        # get_controlMsgs.funtion: 1 << bit0
bit1 = 1
bit2 = 2 


def callback(data):
    mByte = plc2.read_area(readArea, 0, start, length)
    set_bool(mByte, 0, bit0, 0)
    set_bool(mByte, 0, bit1, 0)
    set_bool(mByte, 0, bit2, 0)
    if(data.ReachedHome):
        set_bool(mByte, 0, bit0, 1)
        plc2.write_area(writeArea, 0, start, mByte)
        rospy.loginfo("Reached Home")
        #rospy.sleep(2)
    if(data.ConveyorPlaced):
        set_bool(mByte, 0, bit2, 1)
        plc2.write_area(writeArea, 0, start, mByte)
        rospy.loginfo("Placed on conveyor belt")
        #rospy.sleep(2)
    if(data.DHBWPlaced):
        set_bool(mByte, 0, bit1, 1)
        plc2.write_area(writeArea, 0, start, mByte)
        rospy.loginfo("Placed on DHBW belt")
        #rospy.sleep(2)
    

def controller():
    rospy.init_node("plc_controller_node", anonymous=False)
    pub = rospy.Publisher('/iiwa/plc_control', plcControl, queue_size=10)
    sub = rospy.Subscriber('/iiwa/plc_listner', kukaControl, callback)
    rate = rospy.Rate(30)
    convConter = 0
    dhbwConter = 0
    homeCounter = 0

    while not rospy.is_shutdown():
        # default control messages
        controlMsgs = plcControl()
        controlMsgs.MoveHome = False
        controlMsgs.ConveyorPickPose = False
        controlMsgs.DHBWPickPose = False
    
        mByte = plc.read_area(readArea, 0, start, length)

        if(get_bool(mByte, 0, bit0)):
            controlMsgs.ConveyorPickPose = False
            controlMsgs.DHBWPickPose = False
            controlMsgs.MoveHome = True

        elif(get_bool(mByte, 0, bit1)):
            controlMsgs.MoveHome = False
            controlMsgs.ConveyorPickPose = False
            controlMsgs.DHBWPickPose = True
            rospy.loginfo("Moving IIWA to DHBW Pick Pose")
            
        elif(get_bool(mByte, 0, bit2)):
            controlMsgs.MoveHome = False
            controlMsgs.DHBWPickPose = False
            controlMsgs.ConveyorPickPose = True
            rospy.loginfo("Moving IIWA to Conveyor Pick Pose")
        
        pub.publish(controlMsgs)

        rate.sleep()

        


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInternalException:
        plc.disconnect()
        pass