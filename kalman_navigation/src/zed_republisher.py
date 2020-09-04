#! /usr/bin/env python
import rospy 
from nav_msgs.msg import Odometry


class zed_repub:

    def __init__(self):
        self.sub = rospy.Subscriber("/zed/zed_node/odom", Odometry, self.zed_callback)
        self.pub = rospy.Publisher("/zed/zed_node/odom/repub", Odometry, queue_size=100)

    def zed_callback(self, msg):
        msg.header.frame_id = "odom"
        self.pub.publish(msg)
    

if __name__ == "__main__":

    rospy.init_node("zed_repub")


    zed_repub_instance = zed_repub()

    rospy.spin()