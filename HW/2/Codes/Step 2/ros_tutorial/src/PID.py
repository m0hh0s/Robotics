#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist, Point
import tf
from math import sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np


kp_distance = 4
ki_distance = 0.002
kd_distance = 4

kp_angle = 0.5



class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.last_rotation = 0
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")
                
        
    def run(self, goal_x, goal_y):
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        (position, rotation) = self.get_odom()
        distance_error = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        previous_distance = 0
        total_distance = 0
        while distance_error > 0.5:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y

            angle_error = atan2(goal_y - y_start, goal_x - x_start)
            if angle_error < -pi/4 or angle_error > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    angle_error = -2*pi + angle_error
                elif goal_y >= 0 and y_start > goal_y:
                    angle_error = 2*pi + angle_error
            if self.last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif self.last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            self.last_rotation = rotation
            angle_error -= rotation

            distance_error = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            diff_distance = distance_error - previous_distance
            previous_distance = distance_error
            total_distance += distance_error

            control_signal_distance = kp_distance * distance_error + kd_distance * diff_distance
            control_signal_angle = kp_angle * angle_error

            move_cmd.linear.x = min(control_signal_distance, 0.1)
            move_cmd.angular.z = control_signal_angle
            if control_signal_angle >= 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 0.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -0.5)

            self.cmd_vel.publish(move_cmd)
            r.sleep()
            


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])



if __name__ == '__main__':
    X1 = np.linspace(-3, 3 , 100)
    Y1 = np.array([2]*100)
    Y2 = np.linspace(2, -2 , 100)
    X2 = np.array([3]*100)
    X3 = np.linspace(3, -3 , 100)
    Y3 = np.array([-2]*100)
    Y4 = np.linspace(-2, 2 , 100)
    X4 = np.array([-3]*100)
    shape_x = np.concatenate([X1, X2, X3, X4])
    shape_y = np.concatenate([Y1, Y2, Y3, Y4])
    size = len(shape_x)
    controller = GotoPoint()
    index = 150
    while True:
        while True:
            if index >= size:
                index -= size
            controller.run(shape_x[index], shape_y[index])
            index += 1
