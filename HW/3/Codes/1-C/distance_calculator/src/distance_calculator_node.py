#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from distance_calculator.srv import GetDistance, GetDistanceResponse
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
    def get_distance(self, req):
        obstacle = req.obstacle_name
        position = self.get_position()
        distance = sqrt(pow((obstacles[obstacle][0] - position.x), 2) + pow((obstacles[obstacle][1] - position.y), 2))
        res = GetDistanceResponse()
        res.distance = distance
        return res


    def get_position(self):
        msg = rospy.wait_for_message("/odom" , Odometry) 
        return msg.pose.pose.position


def listener():
    rospy.init_node('distance_calculator_node', anonymous=False)
    dc = Distance_calculator()
    s = rospy.Service('/calc', GetDistance, dc.get_distance)
    rospy.spin()


if __name__ == '__main__':
    listener()