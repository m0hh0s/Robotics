#!/usr/bin/python3

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Closest values in each region
FRONT = 0
LEFT = 0
RIGHT = 0

# Publisher node
CMD_PUB = None


def callback(msg):
    global FRONT, LEFT, RIGHT

    FRONT = min(min(msg.ranges[0:5]), min(msg.ranges[354:359]))
    RIGHT = min(msg.ranges[300:345])
    LEFT = min(msg.ranges[15:60])


def main():
    global CMD_PUB, FRONT, LEFT, RIGHT

    CMD_PUB = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.init_node('maze_solver')

    command = Twist()
    command.linear.x = 0.0
    command.angular.z = 0.0

    rate = rospy.Rate(10)
    time.sleep(1)

    near_wall = 0   # changes to 1 when a wall is found
    distance = 0.5    # distance we keep from the wall

    print("Turning...")
    command.angular.z = -0.5
    command.linear.x = 0.1
    CMD_PUB.publish(command)
    time.sleep(2)

    while not rospy.is_shutdown():
        while(near_wall == 0 and not rospy.is_shutdown()):
            print("Moving towards a wall.")
            if(FRONT > distance and RIGHT > distance and LEFT > distance):  # Nothing there, go straight
                command.angular.z = 0.1
                command.linear.x = 0.3
            elif(LEFT < distance):  # wall found !!
                near_wall = 1
            else:
                command.angular.z = -0.25
                command.linear.x = 0.0

            CMD_PUB.publish(command)

        else:   # left wall detected
            if(FRONT > distance):
                if (LEFT > distance * 9):
                    near_wall = 0
                    print("left wall lost!")
                if(LEFT < (distance / 2)):  # left wall too close
                    print("Too close. Backing up.")
                    command.angular.z = -0.4
                    command.linear.x = -0.1
                elif(LEFT > (distance * 0.75)): # moving forward little to left
                    print("Wall-following")
                    command.angular.z = 1
                    command.linear.x = 0.2
                else:   # moving forward little to right
                    print("Wall-following")
                    command.angular.z = -1
                    command.linear.x = 0.2

            else:  # front is blocked
                print("Front obstacle detected. Turning away.")
                command.angular.z = -1
                command.linear.x = 0.0
                CMD_PUB.publish(command)
                while(FRONT < 0.3 and not rospy.is_shutdown()):
                    CMD_PUB.publish(command)

            CMD_PUB.publish(command)
        rate.sleep()


if __name__ == '__main__':
    main()