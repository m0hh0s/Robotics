#!/usr/bin/python3
import rospy
from distance_calculator.msg import ObstacleDistance
from distance_calculator.srv import GetDistance

obstacles = ['bookshelf',
            'dumpster',
            'barrel',
            'postbox',
            'brick_box',
            'cabinet',
            'cafe_table',
            'fountain'
]


class Main_Node():
    def __init__(self) -> None:
        rospy.init_node('main_node', anonymous=False)
        rate = rospy.Rate(5)
        
        self.obstacle_pub = rospy.Publisher("/ClosestObstacle" , ObstacleDistance , queue_size=10)
        rospy.wait_for_service('/calc')
        self.distance_calc = rospy.ServiceProxy('/calc', GetDistance)

        while True:
            closest_obstacle, distance = self.read_distance()
            msg = ObstacleDistance()
            msg.obstacle_name = closest_obstacle
            msg.distance = distance
            self.obstacle_pub.publish(msg)
            rate.sleep()


    def read_distance(self):
        closest_obstacle = ''
        distance = 100
        for obstacle in obstacles:
            message = self.distance_calc(obstacle)
            obst_distance = message.distance
            if obst_distance < distance:
                distance = obst_distance
                closest_obstacle = obstacle
        return closest_obstacle, distance




if __name__ == '__main__':
    Main_Node()