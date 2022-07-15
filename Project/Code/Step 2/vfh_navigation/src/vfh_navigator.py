#!/usr/bin/python3

import rospy
import numpy
from math import atan2, isinf, degrees, radians, dist, pi
from scipy.signal import argrelextrema
from tf import transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

NUM_RANGES = 360
SECTOR_SIZE = 5
NUM_SECTORS = 72
THRESHOLD = 4.5
MAX_LIN_VEL = 0.6
MAX_ANG_VEL = 0.9
VALLEY_WIDENESS = 6
EPSILON = 0.2


class Navigator:
    def __init__(self):
        rospy.init_node('navigator', anonymous=True)
        rospy.Subscriber('/odom', Odometry, callback=self.update_position)
        rospy.Subscriber('/scan', LaserScan, callback=self.update_histogram)

        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.histogram = [0 for _ in range(NUM_SECTORS)]
        self.a = 1
        self.b = 0.2
    
        self.goal = (rospy.get_param('/vfh_navigator/goal_x'), rospy.get_param('/vfh_navigator/goal_x'))
        self.target_heading = 0
        self.target_distance = 0
        self.viable_heading = 0
        self.position = (0, 0)
        self.heading = 0

        self.rate = rospy.Rate(4)

    def update_histogram(self, scan_data: LaserScan):
        m = [0 for _ in range(NUM_RANGES)]

        for i in range(NUM_RANGES):
            if isinf(scan_data.ranges[i]):
                m[i] = 0
            else:
                m[i] = self.a - self.b * scan_data.ranges[i]
        

        h = [0 for _ in range(NUM_SECTORS)]

        for i in range(NUM_SECTORS):
            h[i] = m[(i * SECTOR_SIZE - 2) % NUM_RANGES] + \
                    m[(i * SECTOR_SIZE - 1) % NUM_RANGES] + \
                    m[i * SECTOR_SIZE] + \
                    m[(i * SECTOR_SIZE + 1) % NUM_RANGES] + \
                    m[(i * SECTOR_SIZE + 2) % NUM_RANGES]
        
        for i in range(NUM_SECTORS):
            self.histogram[i] = h[(i - 2) % NUM_SECTORS] + \
                                2 * h[(i - 1) % NUM_SECTORS] + \
                                2 * h[i] + \
                                2 * h[(i + 1) % NUM_SECTORS] + \
                                h[(i + 2) % NUM_SECTORS]
            self.histogram[i] /= 5
        
    def update_position(self, odom_data: Odometry):
        self.position = (odom_data.pose.pose.position.x, odom_data.pose.pose.position.y)
        orientation = odom_data.pose.pose.orientation
        roll, pitch, yaw = transformations.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w))
        self.heading = yaw
        self.target_distance = dist(self.position, self.goal)
        self.get_target_heading()
    
    def get_target_heading(self):
        goal_heading = atan2(self.goal[1] - self.position[1], self.goal[0] - self.position[0])
        robot_heading = self.heading
        if robot_heading > pi:
            robot_heading -= 2 * pi
        
        self.target_heading = goal_heading - robot_heading
        if self.target_heading < -pi:
            self.target_heading += 2 * pi
        elif self.target_heading > pi:
            self.target_heading -= 2 * pi
        
        self.get_viable_heading()
    
    def get_viable_heading(self):
        target_heading = self.target_heading
        target_sector = int(round(degrees(target_heading) / SECTOR_SIZE))
        if target_sector < 0:
            target_sector += NUM_SECTORS
        
        viable_heading = 0

        threshold = THRESHOLD

        side1_sector = NUM_SECTORS - 1
        side2_sector = 0
        
        # ha = numpy.array(self.histogram)
        # minima = argrelextrema(ha, numpy.less_equal, mode='wrap')

        # if target_sector in minima[0]:
        #     viable_heading = target_heading


        if self.histogram[target_sector] < threshold:
            viable_heading = target_heading
        
        else:
            for i in range(NUM_SECTORS):
                viable = True
                for j in range(VALLEY_WIDENESS):
                    k = target_sector + i + j
                    if self.histogram[k % NUM_SECTORS] > threshold:
                        viable = False
                if viable == True:
                    side1_sector = (target_sector + i) % NUM_SECTORS
                    break
            for i in range(NUM_SECTORS):
                viable = True
                for j in range(VALLEY_WIDENESS):
                    k = target_sector - i - j
                    if self.histogram[k % NUM_SECTORS] > threshold:
                        viable = False
                if viable == True:
                    side2_sector = (target_sector - i) % NUM_SECTORS
                    break
            
            dist1 = side1_sector - target_sector
            if dist1 < 0:
                dist1 += NUM_SECTORS
            dist2 = target_sector - side2_sector
            if dist2 < 0:
                dist2 += NUM_SECTORS
            
            if dist1 < dist2:
                dist1 = radians((2 * dist1 + VALLEY_WIDENESS) / 2 * SECTOR_SIZE)
                viable_heading = target_heading + dist1
                
            else:
                dist2 = radians((2 * dist2 + VALLEY_WIDENESS) / 2 * SECTOR_SIZE)
                viable_heading = target_heading - dist2
            
            if viable_heading < -pi:
                viable_heading += 2 * pi
            elif viable_heading > pi:
                viable_heading -= 2 * pi

        self.viable_heading = viable_heading

        self.orient_robot()

    def orient_robot(self):
        twist = Twist()
        linear_speed = MAX_LIN_VEL * (1 - abs(self.viable_heading) / pi)
        angular_speed = MAX_ANG_VEL * self.viable_heading
        
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        try:
            self.cmd_publisher.publish(twist)
        except rospy.exceptions.ROSException:
            pass

    def run(self):
        rospy.sleep(5)

        while not rospy.is_shutdown() and self.target_distance > EPSILON:

            self.rate.sleep()
        
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_publisher.publish(twist)
        self.cmd_publisher.publish(twist)
        self.cmd_publisher.publish(twist)

if __name__ == '__main__':
    navigator = Navigator()
    navigator.run()