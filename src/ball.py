#! /usr/bin/env python

from utils import *
import numpy as np
import rospy
import actionlib
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point, PointStamped
import tf.transformations as transform
from ur10_armsim.msg import FlyBallAction, FlyBallResult, FlyBallFeedback


class Ball(object):
    """Simulates a ball flying in a straight line"""
    def __init__(self):
        self.position = [1, 1, 1]
        self.speed = 1
        self.tolerance = 0.001
        self.rate = rospy.Rate(50)
        self.pub_world_pos = rospy.Publisher("/ball/world_pos", PointStamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.action_server = actionlib.SimpleActionServer("/ball", FlyBallAction, self.fly_ball_cb, False)
        self.action_server.start()

    def run(self):
        while (not rospy.is_shutdown()):
            self.update_pos(self.position)
            self.rate.sleep()

    def fly_ball_cb(self, goal):
        print "Received fly ball goal %s" % goal
        start = [goal.start.x, goal.start.y, goal.start.z]
        end = [goal.end.x, goal.end.y, goal.end.z]
        self.fly_to(start, end, goal.speed)
        result = FlyBallResult()
        result.final_position.x = self.position[0]
        result.final_position.y = self.position[1]
        result.final_position.z = self.position[2]
        self.action_server.set_succeeded(result)
        print "Ended fly ball with result %s" % result

    def fly_to(self, start, end, speed):
        pos_diff = np.inf
        self.position = start
        while not rospy.is_shutdown() and pos_diff > self.tolerance:
            time_diff = (rospy.Time.now() - self.rate.last_time + self.rate.remaining()).to_sec()
            self.position = self.interpolate_pos(start, end, speed, time_diff)
            self.update_pos(self.position)
            feedback = FlyBallFeedback()
            feedback.world_pos.x = self.position[0]
            feedback.world_pos.y = self.position[1]
            feedback.world_pos.z = self.position[2]
            feedback.distance_to_tcp = self.get_distance_to_tcp(self.position)
            feedback.tcp_pos = self.transform_to_tcp(self.position)
            self.action_server.publish_feedback(feedback)
            pos_diff = np.linalg.norm(np.array(end) - self.position)
            self.rate.sleep()

    def interpolate_pos(self, start, end, speed, delta_t):
        """
        Interpolate the position of the ball between the start and end points
        @param start: start point
        @param end: end point
        @param speed: speed of movement
        @param delta_t: time delta
        @return: a position vector
        """
        vector = np.array(end) - start
        vector = vector / np.linalg.norm(vector)
        pos = np.array(self.position) + vector * speed * delta_t
        return pos

    def update_pos(self, pos):
        """
        Publish the position of the ball
        @param pos: position vector
        @return: None
        """
        point = PointStamped()
        point.header.frame_id = "world"
        point.point.x = pos[0]
        point.point.y = pos[1]
        point.point.z = pos[2]
        self.pub_world_pos.publish(point)

    def get_tcp_pos(self):
        """
        Get the world coordinate of the tcp joint
        @return: position vector
        """
        try:
            tf = self.tf_buffer.lookup_transform("world", "tcp", rospy.Time())
            transform = self.matrix_from_tf(tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            transform = np.identity(4)
        tcp_world_pos = np.matmul(transform, [0, 0, 0, 1])
        return tcp_world_pos[0:3]

    def get_distance_to_tcp(self, pos):
        """
        Calculate the distance to the tcp of pos
        @param pos: position vector
        @return: distance from pos to the tcp position
        """
        tcp = self.get_tcp_pos()
        dist = np.linalg.norm(pos - tcp)
        return dist

    def transform_to_tcp(self, pos):
        """
        Convert world coordinates to the tcp reference frame
        @param pos: position in world frame
        @return: point in tcp frame
        """
        try:
            tf = self.tf_buffer.lookup_transform("tcp", "world", rospy.Time())
            transform = self.matrix_from_tf(tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            transform = np.identity(4)
        pos_h = np.append(pos, 1)
        tcp_pos = np.matmul(transform, pos_h)
        p = Point()
        p.x = tcp_pos[0]
        p.y = tcp_pos[1]
        p.z = tcp_pos[2]
        return p

    def matrix_from_tf(self, tf):
        """
        Returns a transformation matrix from a tf
        @param tf: a tf message
        @return: 4x4 numpy array containing an homogeneous transformation
        """
        quaternion = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
        mat = transform.quaternion_matrix(quaternion)
        translation = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
        mat[0:3, 3] = translation
        return mat

if __name__ == '__main__':
  rospy.init_node("ball")
  node = Ball()
  node.run()
