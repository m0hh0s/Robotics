#!/usr/bin/python3

from dis import dis
import rospy, tf, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import degrees, atan2, pow, sqrt


GOAL_X = 3
GOAL_Y = -1
CMD_PUB = None
LASER_DATA = []

# Closest values in each region
FRONT = 0
LEFT = 0
RIGHT = 0


def main():
    global CMD_PUB
    CMD_PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.init_node('path_planner')
    rate = rospy.Rate(10)
    time.sleep(1)
    while not is_goal_reached():
        go_towards_goal()
        follow_wall(rate)
        rospy.sleep(1)


def follow_wall(rate):
    global CMD_PUB, FRONT, LEFT, RIGHT
    command = Twist()
    command.linear.x = 0.0
    command.angular.z = 0.0
    time.sleep(1)
    near_wall = 0   # changes to 1 when a wall is found
    distance = 0.5    # distance we keep from the wall
    while (is_goal_blocked() and not is_goal_reached()):
        while(near_wall == 0):  # finding a wall
            if(FRONT > distance and RIGHT > distance and LEFT > distance):  # Nothing there, go straight
                command.angular.z = -0.3
                command.linear.x = 0.1
            elif(LEFT < distance):  # wall found !!
                near_wall = 1
            else:
                command.angular.z = -0.25
                command.linear.x = 0.0
            CMD_PUB.publish(command) 

        # left wall detected
        if(FRONT > distance):
            if(LEFT < (distance / 2)):  # left wall too close
                print("Too close. Backing up.")
                command.angular.z = -0.2
                command.linear.x = -0.1
            elif(LEFT > distance): # moving forward little to left
                print("Wall-following")
                command.angular.z = 0.4
                command.linear.x = 0.1
            else:   # moving forward little to right
                print("Wall-following")
                command.angular.z = -0.4
                command.linear.x = 0.1
        else:  # front is blocked
            print("Front obstacle detected. Turning away.")
            command.angular.z = -0.5
            command.linear.x = 0.0
            CMD_PUB.publish(command)
            while FRONT < 0.2:
                CMD_PUB.publish(command)
        CMD_PUB.publish(command)
        rate.sleep()


def go_towards_goal():
    global GOAL_X, GOAL_Y, CMD_PUB, LASER_DATA
    position, orientation = get_odom()
    angle_rad = atan2(GOAL_Y - position.y, GOAL_X - position.x)
    rotate(angle_rad - orientation) # rotate towards goal
    while (not is_goal_blocked() and not is_goal_reached()):
        print("Goal-Seeking")
        twist = Twist()
        twist.linear.x = 0.1
        CMD_PUB.publish(twist)
        rospy.sleep(1)
    CMD_PUB.publish(Twist())
    rospy.sleep(1)


def is_goal_blocked():
    global GOAL_X, GOAL_Y, CMD_PUB, LASER_DATA, FRONT
    position, orientation = get_odom()
    angle_rad = atan2(GOAL_Y - position.y, GOAL_X - position.x) - orientation
    angle_deg = int(degrees(angle_rad))
    if angle_deg < 15:
        distance = FRONT
    else:
        distance = min(LASER_DATA[angle_deg - 15: angle_deg + 15])   # distance to nearest obstacle towards goal
    print("distance to obstacle: " + str(distance))
    if distance > 0.6:
        return False
    return True


def is_goal_reached():
    global GOAL_X, GOAL_Y
    position, orientation = get_odom()
    error = sqrt(pow(GOAL_X - position.x, 2) + pow(GOAL_Y - position.y, 2))
    if (error < 0.1):
        return True
    return False


def laser_callback(msg):
    global LASER_DATA, FRONT, LEFT, RIGHT
    LASER_DATA = msg.ranges
    FRONT = min(min(msg.ranges[0:5]), min(msg.ranges[354:359]))
    RIGHT = min(msg.ranges[300:345])
    LEFT = min(msg.ranges[15:60])


def rotate(goal_angle):
    global CMD_PUB
    print("Rotating")
    turning_sign = 1
    if goal_angle < 0:
        turning_sign = -1
        goal_angle *= -1
    CMD_PUB.publish(Twist())
    rospy.sleep(1)           
    remaining = goal_angle
    prev_angle = get_odom()[1]       
    twist = Twist()
    twist.angular.z = turning_sign * 0.2
    CMD_PUB.publish(twist)            
	# rotation loop
    while remaining > 0.1:
        current_angle = get_odom()[1]
        delta = abs(prev_angle - current_angle)
        remaining -= delta
        prev_angle = current_angle
    CMD_PUB.publish(Twist())
    rospy.sleep(1)  


def get_odom():
    msg = rospy.wait_for_message("/odom" , Odometry)
    orientation = msg.pose.pose.orientation
    # convert quaternion to odom
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((orientation.x ,orientation.y ,orientation.z ,orientation.w))
    return msg.pose.pose.position , yaw


if __name__ == '__main__':
    main()