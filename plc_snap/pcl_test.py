#!/usr/bin/env python

from time import sleep
import snap7 as sn
from snap7.util import *
import struct

plc = sn.client.Client()
plc.connect("192.168.0.1", 0, 1)

area = 0x81    # area for Q memory
start = 200      # location we are going to start the read
length = 1     # length in bytes of the read
bit = 1        # which bit in the Q memory byte we are reading

byte = plc.read_area(area, 0, start, length)
print "Q200.1:",get_bool(byte,0,bit)


set_bool(byte, 0, 1, 1)
plc.write_area(area, 0, 200, byte)

plc.disconnect()