#!/usr/bin/python3

import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import radians


class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller", anonymous=False)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.length = rospy.get_param("/controller/length")
        self.width = rospy.get_param("/controller/width")
        # defining the states of our robot
        self.GO, self.IN_LENGTH = 1, 1


    def get_heading(self):
		# waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry) 
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((orientation.x ,orientation.y ,orientation.z ,orientation.w))
        return yaw
    
    
    def rotate(self):
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)           
        remaining = self.goal_angle
        prev_angle = self.get_heading()          
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)            
		# rotation loop
        while remaining >= 0:
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            remaining -= delta
            prev_angle = current_angle
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)            
    
    
    def go_straight(self):
        traveled = 0
        steps = 0
        if self.IN_LENGTH == 1:
                steps = self.length
                self.IN_LENGTH = 0
        else:
                steps = self.width
                self.IN_LENGTH = 1
        msg = rospy.wait_for_message("/odom" , Odometry) 
        start_x = msg.pose.pose.position.x
        start_y = msg.pose.pose.position.y
        while steps > traveled:
            twist = Twist()
            twist.linear.x = self.linear_speed
            self.cmd_publisher.publish(twist)
            msg = rospy.wait_for_message("/odom" , Odometry) 
            traveled = abs(msg.pose.pose.position.y - start_y) + abs(msg.pose.pose.position.x - start_x)  
        rospy.sleep(1)   
    

    def run(self):
        while not rospy.is_shutdown():
            self.go_straight()
            self.rotate() 
            	

if __name__ == "__main__":
    controller = Controller()
    controller.run()
