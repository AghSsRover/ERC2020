#!/usr/bin/env python


import rospy
import yaml
import rospkg

from ar_track_alvar_msgs.msg import AlvarMarkers
from fiducial_msgs.msg import FiducialTransformArray
from fiducial_msgs.msg import FiducialTransform

from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool
import time


class ARTagsTransformer():
    def __init__(self):
        print("init")
        self.ar_tag_poses_listener = rospy.Subscriber(
            "/ar_pose_marker", AlvarMarkers, self.ar_tag_callback)
        self.fiducial_slam_marker_publisher = rospy.Publisher(
            "/fiducial_transforms", FiducialTransformArray, queue_size=10)

        self.detection_available_publisher = rospy.Publisher(
            "/new_pose", Bool, queue_size=1)

        self.update_pose_srv = rospy.Service(
            "/update_pose", Empty, self.update_pose_handle)

        self.config = None
        # TODO Parametrize it
        self.update_pose_max_wait = 5

        rospack = rospkg.RosPack()
        with open(rospack.get_path('fiducial_slam') + "/cfg/legal_labels.yaml", 'r') as stream:
            try:
                self.config = yaml.safe_load(stream)["legal_labels"]
            except yaml.YAMLError as exc:
                print(exc)

    def marker_id_ok(self, marker):
        print(marker.id)
        print(self.config)
        if marker.id is not None and marker.id in self.config:
            return True
        else:
            return False

    def ar_tag_callback(self, ar_message):
        self.last_ar_message = ar_message
        markers = [marker for marker in ar_message.markers
                   if self.marker_id_ok(marker)]

        if len(markers) > 0:
            self.detection_available_publisher.publish(Bool(True))

    def update_pose_handle(self, req):
        # zeroize to get rid of previous detection
        self.last_ar_message = AlvarMarkers()

        start = time.time()

        while (time.time() - start) < self.update_pose_max_wait:
            ficudial_array = FiducialTransformArray()

            for marker in self.last_ar_message.markers:
                if not self.marker_id_ok(marker):
                    continue

                ficudial_msg = FiducialTransform()
                ficudial_msg.fiducial_id = marker.id
                ficudial_msg.image_error = 0.01  # TODO: ????
                ficudial_msg.object_error = marker.confidence
                ficudial_msg.fiducial_area = 0.2  # TODO: ???
                ficudial_msg.transform.rotation = marker.pose.pose.orientation
                ficudial_msg.transform.translation = marker.pose.pose.position

                ficudial_array.header = marker.header
                ficudial_array.transforms.append(ficudial_msg)

            if len(ficudial_array.transforms) > 0:
                self.fiducial_slam_marker_publisher.publish(ficudial_array)
                break

        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('ar_tag_fiducial_transform')

    qr_tag_bridge = ARTagsTransformer()

    #rate = rospy.Rate(0.5)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     ar_tag_msg = AlvarMarkers()
    #     marker = AlvarMarker()

    #     marker.confidence = 0.8
    #     marker.id = 0
    #     marker.pose.pose.position.x = 2.0
    #     marker.pose.pose.position.y = 1.0
    #     marker.pose.pose.position.z = 1.0
    #     marker.pose.pose.orientation.x = 0.0
    #     marker.pose.pose.orientation.y = 0.0
    #     marker.pose.pose.orientation.z = 0.0
    #     marker.pose.pose.orientation.w = 1.0
    #     ar_tag_msg.markers.append(marker)
    #     print("calling")
    #     rospy.Publisher("/ar_pose_marker", AlvarMarkers, queue_size=1).publish(ar_tag_msg)
    #     # qr_tag_bridge.ar_tag_callback(ar_tag_msg)

    #     rate.sleep()
