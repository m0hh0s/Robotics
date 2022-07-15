#!/usr/bin/python3

import rospy
from rand_stu.msg import Student


def callback(student):
    print(student.name + ' ' + student.last_name + ' is ' + str(student.age) + ' years old and studies ' + student.departement)


def software():
    rospy.Subscriber('software', Student, callback)
    rospy.init_node('software', anonymous=True)
    
    rospy.spin()
        

if __name__ == '__main__':
    try:
        software()
    except rospy.ROSInterruptExeption:
        pass
