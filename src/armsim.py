#! /usr/bin/env python

import utils
import rospy

class ArmSim(object):
    """A basic simulator for a robot arm"""
    def __init__(self):
        # load Denavit_Hartenberg parameters for the links
        self.dh_links = rospy.get_param("dh_links", None)
        self.rate = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            print self.dh_links
            print utils.rotx_matrix(0.1)
            self.rate.sleep()


if __name__ == '__main__':
  rospy.init_node("arm_sim")
  node = ArmSim()
  node.run()