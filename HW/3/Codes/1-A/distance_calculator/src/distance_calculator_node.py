#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from distance_calculator.srv import GetDistance, GetDistanceResponse
from distance_calculator.msg import ObstacleDistance
from math import sqrt


obstacles = {'bookshelf':(2.64,-1.55),
            'dumpster':(1.23, -4.57),
            'barrel':(-2.51, -3.08),
            'postbox':(-4.47, -0.57),
            'brick_box':(-3.44, 2.75),
            'cabinet':(-0.45, 4.05),
            'cafe_table':(1.91, 3.37),
            'fountain':(4.08, 1.14)
}


class Distance_calculator():
    def __init__(self) -> None:
        rospy.init_node('distance_calculator_node', anonymous=False)
        rate = rospy.Rate(5)

        self.obst_pub = rospy.Publisher("ClosestObstacle" , ObstacleDistance , queue_size=10)

        while True:
            closest_obstacle, distance = self.read_distance()
            msg = ObstacleDistance()
            msg.obstacle_name = closest_obstacle
            msg.distance = distance
            self.obst_pub.publish(msg)
            rate.sleep()


    def read_distance(self):
        position = self.get_position()
        closest_obstacle = ''
        distance = 100
        for obstacle in obstacles:
            obst_distance = sqrt(pow((obstacles[obstacle][0] - position.x), 2) + pow((obstacles[obstacle][1] - position.y), 2))
            if obst_distance < distance:
                distance = obst_distance
                closest_obstacle = obstacle
        return closest_obstacle, distance


    def get_position(self):
        msg = rospy.wait_for_message("/odom" , Odometry) 
        return msg.pose.pose.position



if __name__ == '__main__':
    Distance_calculator()