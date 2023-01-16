#! /usr/bin/env python

import rospy

from intera_interface import Head
from sensor_msgs.msg import JointState

class Stable_head(object):

    def __init__(self):

        self._done = False
        self._head = Head()
        self.base_pan = 0
        self.counter = 0

        rospy.Subscriber('/robot/joint_states', JointState, self.joint_angles_callback)
        while not rospy.is_shutdown():

            self._head.set_pan(self.base_pan)

            #print(self.counter)

    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        if self._done:
            self.set_neutral()

    def set_neutral(self):
        """
        Sets the head back into a neutral pose
        """
        self._head.set_pan(0.0)

    def joint_angles_callback(self, msg): 
        print(msg.position)       
        self.base_pan = -0.25 - msg.position[1]

        self.counter += 1
        self.offset = msg.position[0]

def main():
    try:
        rospy.init_node("active_head_py")

        header = Stable_head()

    except rospy.ROSInterruptException:
        self.done = True
        rospy.on_shutdown(header.clean_shutdown)
        rospy.loginfo("Clean Shutdown Done")


if __name__ == '__main__':
    main()
