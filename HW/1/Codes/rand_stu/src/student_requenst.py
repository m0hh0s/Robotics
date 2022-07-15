#!/usr/bin/python3

import rospy
from rand_stu.msg import Student
from random_student import randStudent


def student_request():
    pub = rospy.Publisher('std_request', Student, queue_size=10)
    rospy.init_node('student_request', anonymous=True)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        student = randStudent()
        print('publishing ' + student.name + ' ' + student.last_name)
        pub.publish(student)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        student_request()
    except rospy.ROSInterruptExeption:
        pass
