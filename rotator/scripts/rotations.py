#!/usr/bin/env python

from rotator.msg import RotationAction, RotationResult
import actionlib
import tf2_ros
import rospy
from tf.transformations import euler_from_quaternion
import math

class Server:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('/follow_rotation', RotationAction, self.execute, False)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.previous_transform_time = 0.1

        self.rate = rospy.Rate(1 / self.previous_transform_time)
        self.server.start()

        # we take min of two values and subtract it from angle to give some time to stop rover
        self.fraction_clip = 0.1
        self.angle_clip = 15

    def execute(self, goal):
        result = RotationResult()
        distance_traveled = 0
        
        start = rospy.Time.now()

        angle_to_travel = abs(goal.angle)
        angle_to_travel = self.clip(angle_to_travel)

        print("ang to trav {}".format(angle_to_travel))
        while (abs(distance_traveled) < angle_to_travel):
            # TODO try except
            current_transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
            
            past_transform_time = current_transform.header.stamp - rospy.Duration(self.previous_transform_time)
            previous_transform = self.tf_buffer.lookup_transform("map", "base_link", past_transform_time) 

            previous_yaw = self.to_yaw(previous_transform.transform.rotation)
            current_yaw = self.to_yaw(current_transform.transform.rotation)

            # convert to unit vectors on plane
            previous_vec = [math.cos(previous_yaw), math.sin(previous_yaw)]
            current_vec = [math.cos(current_yaw), math.sin(current_yaw)]

            diff_angle = self.angle_between_vecs(previous_vec, current_vec)
            distance_traveled += diff_angle / math.pi * 180

            print("diff angle {}".format(diff_angle))
            print("distance trav {}".format(distance_traveled))

            now = rospy.Time.now()

            if (now - start).to_sec() > goal.timeout:
                result.success = False

                self.server.set_aborted(result)
                return result

            self.rate.sleep()


        result.success = True
        self.server.set_succeeded(result)
        return result

    def clip(self, angle):
        """
        clip angle to give some time to stop rover
        """
        return angle - min(angle * self.fraction_clip, self.angle_clip)
        

    def to_yaw(self, quaternion):
        yaw, _, _ = euler_from_quaternion([quaternion.w,
                                           quaternion.x,
                                           quaternion.y,
                                           quaternion.z,
                                        ])
        return yaw

    def angle_between_vecs(self, vec1, vec2):
        dot_prod = self.dot(vec1, vec2) / self.norm(vec1) / self.norm(vec2)
        return math.acos(dot_prod)

    def dot(self, x, y):
        """Dot product as sum of list comprehension doing element-wise multiplication"""
        return sum(x_i*y_i for x_i, y_i in zip(x, y))
    
    def norm(self, vec):
        return math.sqrt(sum([i**2 for i in vec]))

if __name__ == "__main__":
    rospy.init_node('rotator')
    Server()
    rospy.spin()
