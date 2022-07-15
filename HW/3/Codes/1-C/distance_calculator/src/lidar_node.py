#!/usr/bin/python3

from attr import has
import rospy, tf
from distance_calculator.msg import ObstacleDistance
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians

angle_goal = 0
turning_sign = 1
cmd_vel = None
has_turned = False


def callback(msg):
    global cmd_vel, angle_goal, turning_sign, has_turned
    if msg.distance < 1.5:
        if not has_turned:
            cmd_vel.publish(Twist())
            rospy.sleep(1)
            remaining = angle_goal
            prev_angle = get_heading()          
            twist = Twist()
            twist.angular.z = turning_sign * 0.2
            cmd_vel.publish(twist)
            while remaining >= 0:
                print("remaining: " + str(remaining))
                current_angle = get_heading()
                delta = abs(prev_angle - current_angle)
                remaining -= delta
                prev_angle = current_angle
            has_turned = True
            cmd_vel.publish(Twist())
            rospy.sleep(1)
    else:
        has_turned = False
    tw = Twist()
    tw.linear.x = 0.2
    cmd_vel.publish(tw)



def listen():
    global cmd_vel

    rospy.init_node('listener', anonymous=False)

    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ClosestObstacle', ObstacleDistance, callback)
    rospy.Subscriber('/scan', LaserScan, read_distance)

    rospy.spin()


def read_distance(data):
    global angle_goal, turning_sign
    ranges = np.array(data.ranges)
    angle_goal = np.argmin(ranges) - 180
    if angle_goal < 0:
        turning_sign = -1
        angle_goal = -1 * angle_goal
    else:
        turning_sign = 1
    angle_goal = radians(angle_goal)


def get_heading():
		# waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry) 
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((orientation.x ,orientation.y ,orientation.z ,orientation.w))
        return yaw
    

if __name__ == '__main__':
    listen()