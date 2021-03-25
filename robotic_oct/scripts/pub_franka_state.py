#! /usr/bin/env python3
'''
publish franka state as ros default message types
'''
import numpy as np
import rospy
import math
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray


def ee_callback(msg):
    EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
    global T_O_ee
    T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12],
                       EE_pos[12:16]]).transpose()


rospy.Subscriber('franka_state_controller/franka_states',
                 FrankaState, ee_callback)

franka_state_pub = rospy.Publisher(
    'franka_state_custom', Float64MultiArray, queue_size=1)

T_O_ee = None


def main():
    global T_O_ee
    rospy.init_node('publish_franka_state', anonymous=True)
    franka_state_msg = Float64MultiArray()
    print("publishing franka state ...")
    while not rospy.is_shutdown():
        if T_O_ee is not None:
            franka_state_msg.data = T_O_ee.flatten()
            franka_state_pub.publish(franka_state_msg)
        # print(T_O_ee)


if __name__ == "__main__":
    main()
