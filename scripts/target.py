#!/usr/bin/env python 
#encoding:utf8 

"""

	it is a stuff list for GPSR 
	it contains the pose of all the stuff
        
"""
from geometry_msgs.msg import *
from geometry_msgs.msg._Pose import Pose


# mode = 1 在桌子上
# mode = 2 在架子上
target={
0x09:{'name': 'SPEAKER', 'pos': Pose(Point(4.817, 1.928, 0.000),Quaternion(0.000, 0.000, 0.016, 1.000)), 'mode': 1 },
0x10:{'name': 'Gray', 'pos': Pose(), 'mode': 1 },
0x11:{'name': 'David', 'pos': Pose(), 'mode': 1 },
0x12:{'name': 'Daniel', 'pos': Pose(), 'mode': 1 },
0x13:{'name': 'Jack', 'pos': Pose(), 'mode': 1 },
0x14:{'name': 'Jenny', 'pos': Pose(), 'mode': 1 },
0x15:{'name': 'Michael', 'pos': Pose(), 'mode': 1 },
0x16:{'name': 'Lucy', 'pos': Pose(), 'mode': 1 },
0x17:{'name': 'Peter', 'pos': Pose(), 'mode': 1 },
0x18:{'name': 'Tom', 'pos': Pose(), 'mode': 1 },
0x19:{'name': 'Jordan', 'pos': Pose(), 'mode': 1 },
0x20:{'name': 'kitchen', 'pos': Pose(Point(4.881, 4.595, 0.000),Quaternion(0.000, 0.000, 0.730, 0.683)), 'mode': 1 },
0x21:{'name': 'living-room', 'pos': Pose(Point(2.297, 1.492, 0.000),Quaternion(0.000, 0.000, 0.066, 0.998)), 'mode': 1 },
0x22:{'name': 'bed-room', 'pos': Pose(Point(9.122, 1.874, 0.000),Quaternion(0.000, 0.000, -0.048, 0.999)), 'mode': 1 },
0x23:{'name': 'dining-room', 'pos': Pose(Point(6.927, 5.776, 0.000),Quaternion(0.000, 0.000, 0.053, 0.999)), 'mode': 1 },
0x30:{'name': 'cooking-table', 'pos': Pose(Point(3.058, 5.605, 0.000),Quaternion(0.000, 0.000, 0.707, 0.708)), 'mode': 1 },
0x35:{'name': 'TV-table', 'pos': Pose(Point(4.138, 2.179, 0.000),Quaternion(0.000, 0.000, -0.639, 0.770)), 'mode': 1 },
0x40:{'name': 'book-cabinet', 'pos': Pose(Point(10.200, 1.910, 0.000),Quaternion(0.000, 0.000, 0.724, 0.690)), 'mode': 1 },
0x45:{'name': 'dining-table', 'pos': Pose(Point(9.470, 3.939, 0.000),Quaternion(0.000, 0.000, 0.681, 0.732)), 'mode': 1 },   #qian
#0x45:{'name': 'dining-table', 'pos': Pose((9.561, 7.486, 0.000),Quaternion(0.000, 0.000, -0.736, 0.677)), 'mode': 1 }, #hou
0x50:{'name': 'sprite', 'pos': Pose(), 'mode': 1 },
0x51:{'name': 'red-bull', 'pos': Pose(), 'mode': 1 },
0x52:{'name': 'milk', 'pos': Pose(), 'mode': 1 },
0x53:{'name': 'tea', 'pos': Pose(), 'mode': 1 },
0x54:{'name': 'juice', 'pos': Pose(), 'mode': 1 },
0x55:{'name': 'coffee', 'pos': Pose(), 'mode': 1 },
0x60:{'name': 'biscuit', 'pos': Pose(), 'mode': 1 },
0x65:{'name': 'chips', 'pos': Pose(), 'mode': 1 },
0x70:{'name': 'roll-paper', 'pos': Pose(), 'mode': 1 },
0x71:{'name': 'toothpaste', 'pos': Pose(), 'mode': 1 },
}
