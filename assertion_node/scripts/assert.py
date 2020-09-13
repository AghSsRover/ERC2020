import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, CameraInfo
TOPICS = {
    "/odometry/filtered" : Odometry,
    "/zed2/odom" : Odometry,
    "/zed2/imu/data" : Imu,
    "/zed2/left_raw/camera_info" : CameraInfo
}

def assert_topic(topic, type_):
    try:
        rospy.wait_for_message(topic, type_, rospy.Duration(1.2))
        return True
    except rospy.exceptions.ROSException:
        return False


if __name__ == "__main__":
    rospy.init_node('assert')

    rospy.loginfo("")
    rospy.loginfo("")

    
    for topic in TOPICS.keys():
        rospy.loginfo("topic to check: {}".format( topic))

    rospy.loginfo("")
    rospy.loginfo("")

    rospy.loginfo("------------")
    rospy.loginfo("------------")
    rospy.loginfo("------------")

    rospy.loginfo("")
    rospy.loginfo("")

        
    topics_up = []
    topics_down = []

    for topic, type_ in TOPICS.items():
        if assert_topic(topic, type_):
            topics_up.append(topic)
        else:
            rospy.logerr("{} is down".format(topic))
            rospy.logerr("".format(topic))
            topics_down.append(topic)
    
    
    if len(topics_down) > 0:
        rospy.logerr("SOME TOPICS ARE DOWN")
        rospy.logerr("SOME TOPICS ARE DOWN")
        rospy.logerr("SOME TOPICS ARE DOWN")
        rospy.logerr("SOME TOPICS ARE DOWN")
    else:
        rospy.logdebug("all topics ok")

    rospy.loginfo("")
    rospy.loginfo("")
