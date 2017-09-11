#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from uchile_workspace_analysis.msg import GraspStorage
from uchile_workspace_analysis.srv import GetCapabilityMap, GetCapabilityMapRequest

__author__ = 'Rodrigo Munoz'
__email__ = 'rorro.mr@gmail.com'


class CapabilityMap(object):
    def __init__(self, capability_map_topic="/capability_map"):
        self.capability_map_topic = capability_map_topic

    def get_topic(self):
        return self.capability_map_topic

    def setup(self):
        self.grasp_server = rospy.ServiceProxy(self.capability_map_topic, GetCapabilityMap)
        return True

    def check(self, timeout=1.0):
        # Try to get service connection
        try:
            rospy.wait_for_service(self.capability_map_topic, timeout=timeout)
        except:
            self.logerr("Servicie \"{0}\" not found.".format(self.self.capability_map_topic))
            return False
        return True

    def get_grasp(self, object, group_name, generate_online=False):
        req = GetCapabilityMapRequest()
        req.object = object
        req.group_name = group_name
        req.generate_online = generate_online
        try:
            result = self.grasp_server(req)
            return result.grasp
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return GraspStorage()
