#!/usr/bin/env python


import rospy 

from std_msgs.msg import Header
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform

class ARTagsTransformer():
    def __init__(self):
        print("init")
        self.ar_tag_poses_listener = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_tag_callback)
        self.fiducial_slam_marker_publisher = rospy.Publisher("/fiducial_transforms", FiducialTransformArray, queue_size=10)

    def ar_tag_callback(self, ar_message):
        print("received ar pose")

        ficudial_array = FiducialTransformArray()
        
        ficudial_msg = FiducialTransform()
        ficudial_msg.fiducial_id = ar_message.markers[0].id
        ficudial_msg.image_error = ar_message.markers[0].confidence
        ficudial_msg.object_error = 0.01  #TODO: ????
        ficudial_msg.fiducial_area = 0.2 #TODO: ???
        ficudial_msg.transform.rotation = ar_message.markers[0].pose.pose.orientation
        ficudial_msg.transform.translation = ar_message.markers[0].pose.pose.position

        ficudial_array.header = Header()
        ficudial_array.header.frame_id = "base_link" #TODO:
        ficudial_array.transforms.append(ficudial_msg)

        print(ficudial_array)

        self.fiducial_slam_marker_publisher.publish(ficudial_array)


if __name__ == '__main__':
    rospy.init_node('ar_tag_fiducial_transform')

    qr_tag_bridge = ARTagsTransformer()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        ar_tag_msg = AlvarMarkers()
        marker = AlvarMarker()
        
        marker.confidence = 0.8
        marker.id = 0
        marker.pose.pose.position.x = 2.0
        marker.pose.pose.position.y = 1.0
        marker.pose.pose.position.y = 1.0
        marker.pose.pose.orientation.x = 0.0
        marker.pose.pose.orientation.y = 0.0
        marker.pose.pose.orientation.z = 0.0
        marker.pose.pose.orientation.w = 1.0
        ar_tag_msg.markers.append(marker)
        print("calling")
        rospy.Publisher("/ar_pose_marker", AlvarMarkers, queue_size=1).publish(ar_tag_msg)
        # qr_tag_bridge.ar_tag_callback(ar_tag_msg)

        rate.sleep()
