#! /usr/bin/env python

from utils import *
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations as transform


class ArmSim(object):
    """A basic simulator for a robot arm"""
    def __init__(self):
        # load Denavit-Hartenberg parameters for the links
        self.dh_links = rospy.get_param("dh_links", [])
        # load link names
        self.link_names = ["world"] + rospy.get_param("link_names", [])
        self.max_speeds = rospy.get_param("link_max_speeds", [])
        self.initial_state = rospy.get_param("link_initial", [])
        self.rate = rospy.Rate(50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def run(self):
        #target_angles = (np.random.rand(6) - 0.5) * np.pi * 2
        target_angles = [np.pi, 0, 0, 0, 0,0]
        angles = self.initial_state
        k = 0.02
        tol = 0.001
        while not rospy.is_shutdown():
            time_diff = (rospy.Time.now() - self.rate.last_time + self.rate.remaining()).to_sec()
            print "time diff: %s" % time_diff
            for a in range(len(angles)):
                angles[a] += self.angular_control(angles[a], target_angles[a], self.max_speeds[a], time_diff, k, tol)
            print target_angles
            print angles
            self.update_links(angles)
            self.rate.sleep()

    def angular_control(self, current_angle, target_angle, max_speed, time_delta, p_const, tolerance):
        """
        Calculates the angular increment to reach towards the target angle
        @param current_angle: current angle value in radians
        @param target_angle: target angle value in radians
        @param max_speed: maximum angular speed allowed (in radians/sec)
        @param time_delta: time increment to consider (in seconds)
        @param p_const: constant for the proportional control
        @param tolerance: minimum difference allowed for the angle control
        @return: the angular increment to add to the current angle (in radians)
        """
        diff = target_angle - current_angle
        if abs(diff) > tolerance:
            increment = diff * p_const
            max_increment = abs(max_speed * time_delta)
            if abs(increment) > max_increment:
                # clip the increment according to the maximum speed
                print "*** LIMITING SPEED *** %s" % increment
                increment = np.sign(increment) * max_increment
        else:
            increment = 0
        return increment

    def get_link_transform(self, dh_params, angle):
        """
        Returns the transformation matrix for the link specified by its Denavit-Hartenberg parameters
        and the angle value for the joint rotation
        @param dh_params: Denavit-Hartenberg parameters vector
        @param angle: angular value for the joint rotation, in radians
        @return: a 4x4 transformation matrix
        """
        z_translation = dh_params[1]
        x_translation = dh_params[2]
        x_angle = dh_params[3]
        m1 = np.matmul(rotz_matrix(angle), trans_matrix([0, 0, z_translation]))
        m2 = np.matmul(m1, trans_matrix([x_translation, 0, 0]))
        mat = np.matmul(m2, rotx_matrix(x_angle))
        return mat

    def update_links(self, angles):
        """
        Updates the state of the links given the joint angles
        @param angles: vector of joint angles
        @return: None
        """
        for link in range(len(angles)):
            link_mat = self.get_link_transform(self.dh_links[link], angles[link])
            msg = self.tf_from_matrix(link_mat)
            msg.header.frame_id = self.link_names[link]
            msg.child_frame_id = self.link_names[link + 1]
            self.tf_broadcaster.sendTransform(msg)

    def tf_from_matrix(self, transform_mat):
        """
        Returns a tf message from the transformation matrix
        @param transform_mat: 4x4 numpy array containing an homogeneous transformation
        @return: a tf message
        """
        rotation = transform_mat.copy()
        rotation[0:4, 3] = [0, 0, 0, 1]
        q = transform.quaternion_from_matrix(rotation)
        msg = TransformStamped()
        msg.transform.translation.x = transform_mat[0, 3]
        msg.transform.translation.y = transform_mat[1, 3]
        msg.transform.translation.z = transform_mat[2, 3]
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        return msg

if __name__ == '__main__':
  rospy.init_node("arm_sim")
  node = ArmSim()
  node.run()
