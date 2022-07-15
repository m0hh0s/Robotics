#!/usr/bin/python3

import rospy
from rand_stu.msg import Student


pub_h = None
pub_s = None



def callback(student):
    print('splitting ' + student.name + ' ' + student.last_name)
    if student.departement == 'Hardware':
        pub_h.publish(student)
    else:
        pub_s.publish(student)


def splitter():
    global pub_h, pub_s
    pub_h = rospy.Publisher('hardware', Student, queue_size=10)
    pub_s = rospy.Publisher('software', Student, queue_size=10)
    rospy.Subscriber('std_request', Student, callback)
    rospy.init_node('splitter', anonymous=True)
    
    rospy.spin()
        

if __name__ == '__main__':
    try:
        splitter()
    except rospy.ROSInterruptExeption:
        pass
