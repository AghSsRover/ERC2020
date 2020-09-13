#!/usr/bin/env python


import rospy
import yaml
import rospkg

from ar_track_alvar_msgs.msg import AlvarMarkers
from fiducial_msgs.srv import FiducialTransformSrv, FiducialTransformSrvRequest 
from fiducial_msgs.msg import (
    FiducialTransform, 
    FiducialTransformArray,
    FiducialDistance
)
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

from fiducial_slam.srv import (
    UpdatePoseFromTrans, UpdatePoseFromTransResponse,
    UpdatePoseFromRot, UpdatePoseFromRotResponse)

from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool
import time

import tf2_ros
import tf
import numpy as np

from tf2_geometry_msgs import PoseStamped

def to_matrix(transform):
    t = transform.transform.translation
    r = transform.transform.rotation
    tm = tf.transformations.translation_matrix((t.x, t.y, t.z))
    rm = tf.transformations.quaternion_matrix((r.x, r.y, r.z, r.w))

    return np.dot(tm, rm)


class ARTagsTransformer():
    def __init__(self):
        print("init")
        self.ar_tag_poses_listener = rospy.Subscriber(
            "/ar_pose_marker", AlvarMarkers, self.ar_tag_callback)
        self.fiducial_slam_marker_publisher = rospy.ServiceProxy(
            "/fiducial_transforms", FiducialTransformSrv)

        self.detection_available_publisher = rospy.Publisher(
            "/new_pose", Bool, queue_size=1)


        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

#         self.update_pose_orient_srv = rospy.Service(
#             "/update_pose", Empty, self.update_pose_perfect_orientation_handle)

#         self.update_pose_trans_srv = rospy.Service(
#             "/update_pose_perfect_translation",
#             UpdatePoseFromTrans, self.update_pose_perfect_translation_handle)

        self.observed_tags_pub = rospy.Publisher("/observed_landmarks", FiducialDistance, queue_size=10)

        self.update_pose_from_rot_srv = rospy.Service(
            "/update_pose_from_rotation", UpdatePoseFromRot, self.update_pose_perfect_orientation_handle)

        self.update_pose_from_trans_srv = rospy.Service(
            "/update_pose_from_translation", UpdatePoseFromTrans, self.update_pose_perfect_translation_handle)

        self.config = None
        # TODO Parametrize it
        self.update_pose_max_wait = 3.5

        # some stupid lock to avoid updating pose on two different ways simultaneously
        self.lock = False

        rospack = rospkg.RosPack()
        with open(rospack.get_path('fiducial_slam') + "/cfg/legal_labels.yaml", 'r') as stream:
            try:
                self.config = yaml.safe_load(stream)["legal_labels"]
            except yaml.YAMLError as exc:
                print(exc)

    def marker_id_ok(self, marker):

        if marker.id is not None and marker.id in self.config:
            pass
        else:
            return False

        # if self.tfBuffer.canTransform("base_link", marker.header.frame_id, rospy.Time.now()):
        # base_link = self.tfBuffer.lookup_transform("zed2_left_camera_optical_frame",
        #                                         "base_link",
        #                                         rospy.Time.now())

        t = PoseStamped()
        t.header.frame_id = "zed2_left_camera_optical_frame"
        t.pose.position = marker.pose.pose.position
        t.pose.orientation = marker.pose.pose.orientation 

        tag_pose = self.tfBuffer.transform(t, "base_link")

        distance = (tag_pose.pose.position.x**2 + tag_pose.pose.position.y**2)**0.5
        if 0.2 < distance < 8.0:
            header = Header()
            header.frame_id = "base_link"
            header.stamp = rospy.Time.now()
            observed_fiducial = FiducialDistance(distance=distance, id=marker.id, header=header)
            self.observed_tags_pub.publish(observed_fiducial)
            return True
        else:
            return False



    def ar_tag_callback(self, ar_message):
        self.last_ar_message = ar_message
        markers = [marker for marker in ar_message.markers
                   if self.marker_id_ok(marker)]
        if len(markers) > 0:
            # print(markers)
            self.detection_available_publisher.publish(Bool(True))


    # assume that rotation from map to base_link is without any error
    def update_pose_perfect_translation_handle(self, req):
        rospy.loginfo('started service')
        response = UpdatePoseFromTransResponse()
        if self.lock:
            response.success = False
            return response.success

        self.set_lock(True)

        # zeroize to get rid of previous detection
        self.last_ar_message = AlvarMarkers()

        start = time.time()

        while (time.time() - start) < self.update_pose_max_wait:
            update_map_request = FiducialTransformSrvRequest()

            ficudial_array = FiducialTransformArray()

            for marker in self.last_ar_message.markers:
                if not self.marker_id_ok(marker):
                    continue

                ficudial_msg = self._marker_to_fiducial(marker)
                ficudial_array.header = marker.header
                ficudial_array.transforms.append(ficudial_msg)

            if len(ficudial_array.transforms) > 0:
                rospy.loginfo('found fiducials')

                ficudial_array.translation_perfect = True
                ficudial_array.translation = req.translation

                update_map_request.transforms = ficudial_array
                response.success = self.fiducial_slam_marker_publisher.call(update_map_request).success
                rospy.loginfo('map service reponse {}'.format(response.success))
                break

        # TODO SERVICE INSTEAD PUBLISHER
        self.set_lock(False)

        return response.success

    def set_lock(self, boolean):
        self.lock = boolean
    # update robot pose basing on assumption that translation given by user is perfect

    def update_pose_perfect_orientation_handle(self, req):
        response = UpdatePoseFromRotResponse()
        rospy.loginfo("starting updating pose from perfect orientation")
        # zeroize to get rid of previous detection
        if self.lock:
            response.success = False
            return response.success

        self.set_lock(True)

        self.last_ar_message = AlvarMarkers()

        start = time.time()

        while (time.time() - start) < self.update_pose_max_wait:
            update_map_request = FiducialTransformSrvRequest()

            ficudial_array = FiducialTransformArray()

            for marker in self.last_ar_message.markers:
                if not self.marker_id_ok(marker):
                    continue

                ficudial_msg = self._marker_to_fiducial(marker)
                ficudial_array.header = marker.header
                ficudial_array.transforms.append(ficudial_msg)

            if len(ficudial_array.transforms) > 0:
                ficudial_array.translation_perfect = False
                update_map_request.transforms = ficudial_array
                response.success = self.fiducial_slam_marker_publisher.call(update_map_request).success
                rospy.loginfo("fiducial slam response {}".format(response.success))
                rospy.Rate(1).sleep()
                break

        self.set_lock(False)

        return response.success

    @ staticmethod
    def _marker_to_fiducial(marker):
        ficudial_msg = FiducialTransform()
        ficudial_msg.fiducial_id = marker.id
        ficudial_msg.image_error = 0.01  # TODO: ????
        ficudial_msg.object_error = marker.confidence
        ficudial_msg.fiducial_area = 0.2  # TODO: ???
        ficudial_msg.transform.rotation = marker.pose.pose.orientation
        ficudial_msg.transform.translation = marker.pose.pose.position

        return ficudial_msg


if __name__ == '__main__':
    rospy.init_node('ar_tag_fiducial_transform')

    qr_tag_bridge = ARTagsTransformer()

    # rate = rospy.Rate(0.5)
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
