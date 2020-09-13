#! /usr/bin/env python
import rospy 
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovarianceStamped


class wheel_repub:

    def __init__(self):
        self.sub = rospy.Subscriber("/wheel_odom", TwistStamped, self.wheel_callback)
        self.pub = rospy.Publisher("/wheel_odom_covariance", TwistWithCovarianceStamped, queue_size=100)

    def wheel_callback(self, msg):
        twcs = TwistWithCovarianceStamped()
        twcs.header = msg.header
        twcs.twist.twist = msg.twist
        twcs.twist.covariance = [0, 0, 0, 0, 0, 0,\
                          0, 0, 0 ,0 ,0 ,0,\
                          0, 0, 0, 0, 0, 0,\
                          0, 0, 0, 0, 0, 0,\
                          0, 0, 0, 0, 0, 0,\
                          0, 0, 0, 0, 0, 0,]

        self.pub.publish(twcs)
    

if __name__ == "__main__":

    rospy.init_node("wheel_repub")


    wheel_repubinstance = wheel_repub()

    rospy.spin()