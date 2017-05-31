#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Rodrigo Munoz'
__email__ = 'rorro.mr@gmail.com'


import rospy
from hb_workspace_analysis.msg import GraspStorage
from hb_workspace_analysis.srv import GetCapabilityMap, GetCapabilityMapRequest, CapabilityMapResponse

class WorkspaceAnalysis(object):

    def __init__(self):
        self.capability_map_topic = "/capability_map"

    def setup(self):
        self.grasp_server = rospy.ServiceProxy('/capability_map', GetCapabilityMap)

    def get_grasp(self, object):
        req = GetCapabilityMapRequest()
        req.object = object
        try:
            result = self.grasp_server(req)
            return result.grasp
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return GraspStorage()
