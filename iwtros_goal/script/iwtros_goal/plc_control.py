#!/usr/bin/env python
import rospy
from iwtros_msgs.msg import plcControl

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

mbyte = plc.read_area(area, 0, start, length)
print "Q0.0:",get_bool(mbyte,0,bit)
set_bool(mbyte, 0, bit, 1)
print "Q0.0:",get_bool(mbyte,0,bit)
plc.write_area(area_2, 0, start, mbyte)
plc.disconnect()

def callback(data):
    a = 1 + 2
    

def controller():
    rospy.init_node("plc_controller_node", anonymous=False)
    pub = rospy.Publisher('plc_control', plcControl, queue_size=10)
    sub = rospy.Subscriber('plc_listener', plcControl, callback=callback)

    controlMsgs = plcControl
    index0_value = 1 << bit0
    index1_value = 1 << bit1
    index2_value = 1 << bit2

    while not rospy.is_shutdown():
        mByte = plc.read_area(readArea, 0, start, length)
        byte_value = mByte[0]

        if(byte_value & index0_value):
            controlMsgs.HomePosition = True
            controlMsgs.ConveyorPrePickPose = False
            controlMsgs.ConveyorPick = False
            controlMsgs.DHBWPrePlace = False
            controlMsgs.DHBWPlaced = False
            controlMsgs.DHBWPrePickPose = False
            controlMsgs.DHBWPick = False
            controlMsgs.ConveyorPrePlace = False
            controlMsgs.ConveyorPlaced = False
            pub.publish(controlMsgs)

        if(byte_value & index1_value)
        if(byte_value & index2_value)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInternalException:
        plc.disconnect()
        pass