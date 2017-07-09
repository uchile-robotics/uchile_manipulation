#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from hb_workspace_analysis.msg import GraspStorage
from hb_workspace_analysis.srv import GetCapabilityMap, GetCapabilityMapRequest
from hb_workspace_analysis.srv import GetBestBasePose, GetBestBasePoseRequest, GetBestBasePoseResponse

__author__ = 'Rodrigo Munoz'
__email__ = 'rorro.mr@gmail.com'


class CapabilityMap(object):

    CAPABILITY_MAP_SERVICE = "/capability_map"

    def __init__(self):
        self.grasp_server = None

    def get_topic(self):
        return self.CAPABILITY_MAP_SERVICE

    def setup(self):
        self.grasp_server = rospy.ServiceProxy(self.CAPABILITY_MAP_SERVICE, GetCapabilityMap)
        return True

    def check(self, timeout=1.0):
        # Try to get service connection
        try:
            rospy.wait_for_service(self.CAPABILITY_MAP_SERVICE, timeout=timeout)
        except:
            rospy.logerr("Servicie \"{0}\" not found.".format(self.CAPABILITY_MAP_SERVICE))
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

class BestBasePose(object):

    BEST_BASE_POSE_SERVICE = "/best_base_pose"

    def __init__(self):
        self.base_pose_service = None

    def get_topic(self):
        return self.BEST_BASE_POSE_SERVICE

    def setup(self):
        self.base_pose_service = rospy.ServiceProxy(self.BEST_BASE_POSE_SERVICE, GetBestBasePose)
        return True

    def check(self, timeout=1.0):
        # Try to get service connection
        try:
            rospy.wait_for_service(self.BEST_BASE_POSE_SERVICE, timeout=timeout)
        except:
            self.logerr("Servicie \"{0}\" not found.".format(self.BEST_BASE_POSE_SERVICE))
            return False
        return True

    def get_pose(self, object):
        req = GetBestBasePoseRequest()
        req.object = object
        try:
            result = self.base_pose_service(req)
            return result
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return GetBestBasePoseResponse()
