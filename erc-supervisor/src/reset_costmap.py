#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client
from std_srvs.srv import Empty

class CostmapCaretaker:

    def __init__(self):
        self.reset_service = rospy.Service("/reset_costmap", Empty, self.reset_costmap)
        self.resume_service = rospy.Service("/resume_costmap", Empty, self.resume_costmap)
        self.clear_service = rospy.Service("/clear_costmap", Empty, self.clear_costmap)

    def reset_costmap(self, req):
        reset_height_global_client = dynamic_reconfigure.client.Client(
            "/move_base/global_costmap/obstacles_laser/", timeout=5, config_callback=self.dynamic_recofigure_callback)

        reset_height_local_client = dynamic_reconfigure.client.Client(
            "/move_base/local_costmap/obstacles_laser/", timeout=5, config_callback=self.dynamic_recofigure_callback)

        reset_height_local_client2 = dynamic_reconfigure.client.Client(
            "/move_base/local_costmap/obstacles_laser2/", timeout=5, config_callback=self.dynamic_recofigure_callback)

        reset_height_global_client.update_configuration({"max_obstacle_height": 0})
        reset_height_local_client.update_configuration({"max_obstacle_height": 0})
        reset_height_local_client2.update_configuration({"max_obstacle_height": 0})
        print("RESETING ALL COSTMAPS!")
        try:
            clear_costmap = rospy.ServiceProxy(
                "/move_base/clear_costmaps", Empty)
            clear_costmap()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        
        return []

    def resume_costmap(self, req):
        reset_height_global_client = dynamic_reconfigure.client.Client(
            "/move_base/global_costmap/obstacles_laser/", timeout=5, config_callback=self.dynamic_recofigure_callback)

        reset_height_local_client = dynamic_reconfigure.client.Client(
            "/move_base/local_costmap/obstacles_laser/", timeout=5, config_callback=self.dynamic_recofigure_callback)

        reset_height_local_client2 = dynamic_reconfigure.client.Client(
            "/move_base/local_costmap/obstacles_laser2/", timeout=5, config_callback=self.dynamic_recofigure_callback)

        reset_height_global_client.update_configuration({"max_obstacle_height": 0.5})
        reset_height_local_client.update_configuration({"max_obstacle_height": 0.5})
        reset_height_local_client2.update_configuration({"max_obstacle_height": 0.5})
    
        return []


    def clear_costmap(self, req):
        try:
            clear_costmap = rospy.ServiceProxy(
                "/move_base/clear_costmaps", Empty)
            clear_costmap()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        
        return []

    def dynamic_recofigure_callback(self, config):
        rospy.loginfo("Costmap cleared")


if __name__ == "__main__":
    rospy.init_node('costmap_caretaker')
    CostmapCaretaker()
    rospy.loginfo("Costmap caretaker spanwned")
    rospy.spin()
