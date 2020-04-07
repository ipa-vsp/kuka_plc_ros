#!/usr/bin/env python
import rospy
from iwtros_msgs.msg import plcControl 
from iwtros_msgs.msg import kukaControl

from time import sleep
import snap7 as sn
from snap7.util import *
import struct

READ_AREA = 0x82     #area for Q memory
WRITE_AREA = 0x81    #area for I memory
START = 200
LENGTH = 1
BIT0 = 0
BIT1 = 1
BIT2 = 2

class _Control(object):
    def __init__(self, *args, **kwargs):
        super(_Control, self).__init__(*args, **kwargs)
        controllerTopic = "/iiwa/plc_control"
        listenerTopic = "/iiwa/plc_listener"
        self.pub = rospy.Publisher(controllerTopic, plcControl, queue_size=10)
        self.sub = rospy.Subscriber(listenerTopic, kukaControl, self.Callback)

        self._plc_read = sn.client.Client()
        self._plc_read.connect("192.168.0.1", 0, 1)
        self._plc_write = sn.client.Client()
        self._plc_write.connect("192.168.0.1", 0, 1)

        self.waitForUpdate = False
        return


    def Callback(self, data):
        mByte = self._plc_write.read_area(READ_AREA, 0, START, LENGTH)
        set_bool(mByte, 0, BIT0, 0)
        set_bool(mByte, 0, BIT1, 0)
        set_bool(mByte, 0, BIT2, 0)
        if(data.ReachedHome):
            set_bool(mByte, 0, BYTE0, 1)
            self._plc_write(WRITE_AREA, 0, START, mByte)
            rospy.loginfo("Reached Home")
        if(data.ConveyorPlaced):
            set_bool(mByte, 0, BIT2, 1)
            self._plc_write(WRITE_AREA, 0, START, mByte)
            rospy.loginfo("Placed on conveyor belt")
            #rospy.sleep(2)
        if(data.DHBWPlaced):
            set_bool(mByte, 0, BIT1, 1)
            self._plc_write(WRITE_AREA, 0, START, mByte)
            rospy.loginfo("Placed on DHBW belt")

    def __del__(self):
        self._plc_read.destroy()
        self._plc_write.destroy()
    
    def _cntr_loop(self):
        cntrMsg = plcControl()
        while not rospy.is_shutdown():
            cntrMsg.MoveHome = False
            cntrMsg.ConveyorPickPose = False
            cntrMsg.DHBWPickPose = False

            mByte = self._plc_read(READ_AREA, 0, START, LENGTH)

            if(get_bool(mByte, 0, BIT0) and not self.waitForUpdate):
                cntrMsg.MoveHome = True
                self.waitForUpdate = True
                rospy.loginfo("Moving Home")
            elif(get_bool(mByte, 0, BIT1) and not self.waitForUpdate):
                cntrMsg.DHBWPickPose = True
                self.waitForUpdate = True
                rospy.loginfo("Picking From Conveyor belt and Placing in DHBW")
            elif(get_bool(mByte, 0, BIT2) and not self.waitForUpdate):
                cntrMsg.ConveyorPickPose = True
                self.waitForUpdate = True
                rospy.loginfo("Picking from DHBW and Placing in Conveyor belt")


if __name__ == '__main__':
    rospy.init_node("plc_control_node", anonymous=False)
    con = _Control()
    try:
        con._cntr_loop()
    except rospy.ROSInternalException:
        del con
        pass