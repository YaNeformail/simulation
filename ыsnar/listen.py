#!/usr/bin/env python
import rospy
from snar.msg import Num
def callback(data):
    rospy.loginfo(f'\nleft encoder: {data.l}\nright encoder: {data.r}')

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("topic", Num, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
