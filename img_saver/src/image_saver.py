#! /usr/bin/env python

import sys
import os
from datetime import datetime

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import cv2
from time import sleep
from std_srvs.srv import Trigger

class Saver:
    def __init__(self):
        self.im1_sub = rospy.Subscriber(rospy.get_param('~camera_topic1'),
                                        CompressedImage, self.cb_1)
        self.im2_sub = rospy.Subscriber(rospy.get_param('~camera_topic2'),
                                        CompressedImage, self.cb_2)

        self.im1 = CompressedImage()
        self.im2 = CompressedImage()
        self.bridge = CvBridge()
        self.save_srv = rospy.Service("/save_image", Trigger, self.save_callback)

    def cb_1(self, msg):
        self.im1 = msg

    def cb_2(self, msg):
        self.im2 = msg

    def save_callback(self, req):
        response1 = self._save_img(self.im1, 1)
        response2 = self._save_img(self.im2, 2)
        success = (response1 and response2)
        if success:
            message = "saved images"
        else:
            message = "could not save images"
        return  success,message

    def _save_img(self, msg, cam_id):
        try:
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Failed to convert image: %s" % (e,))
            return False

        img_time = datetime.fromtimestamp(msg.header.stamp.secs)
        time_str = img_time.strftime("%Y-%m-%d-%H-%M-%S") + "_" + str(cam_id)
        img_filename = 'image_%s.png' % (time_str,)

        cv2.imwrite(img_filename, cv2_img)

        rospy.loginfo("Saved image from %s topic to %s file", msg.data,
                      os.path.join(os.getcwd(), img_filename))
        return True


if __name__ == "__main__":
    rospy.init_node('image_saver')
    Saver()
    rospy.spin()
