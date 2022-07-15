#!/usr/bin/python3

import rospy
from rand_stu.msg import Student


def callback(student):
    print(student.name + ' ' + student.last_name + ' is ' + str(student.age) + ' years old and studies ' + student.departement)


def hardware():
    rospy.Subscriber('hardware', Student, callback)
    rospy.init_node('hardware', anonymous=True)
    
    rospy.spin()
        

if __name__ == '__main__':
    try:
        hardware()
    except rospy.ROSInterruptExeption:
        pass
