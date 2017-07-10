#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Rodrigo Munoz'
__email__ = 'rorro.mr@gmail.com'


import rospy
import sys
from random import random
from tf import transformations
# Msgs
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import JointState

# Markers
from visualization_msgs.msg import InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker, InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler
from shape_msgs.msg import SolidPrimitive
from hb_workspace_analysis.msg import GraspObject
from hb_workspace_analysis.interface import CapabilityMap

from bender_skills.capabilities.manipulation.capability_map import CapabilityMapSkill

from random import shuffle

def get_pose(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    q = transformations.quaternion_from_euler(roll, pitch, yaw)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

def get_cylinder(pose, frame_id='bender/base_link', dim = [0.22, 0.04]):
    obj = GraspObject()
    obj.header.stamp = rospy.Time.now()
    obj.header.frame_id = frame_id
    cylinder = SolidPrimitive()
    cylinder.type = SolidPrimitive.CYLINDER
    cylinder.dimensions = dim
    obj.primitives.append(cylinder)
    obj.primitives.append(cylinder)
    obj.primitive_poses.append(pose)
    return obj

class GraspTest(object):

    def __init__(self, frame_id = "base_link", arm_name = 'l_arm ',radius = 0.04, height = 0.22, init_position = Point(0,0,0)):
        
        self.current_pose = Pose()
        self.current_pose.position = init_position

        self.radius = radius
        self.height = height
        self.frame_id = frame_id

        self.grasp_server = CapabilityMapSkill()
        if not self.grasp_server.check():
            rospy.logerr(" Can't capability map server at \"{}\"".format(self.grasp_server.get_topic()))
            sys.exit(-1)
        self.grasp_server.setup()



    def process_feedback(self, feedback):
        # Grasp
            
        object = get_cylinder(self.current_pose)
        try:
            result = self.grasp_server.get_grasp(object)
            if result.grasp:
                # Show at least 10 random grasp position
                shuffle(result.grasp)
                i = 0
                for idx in xrange(min(10, len(result.grasp))):
                    self.joint_msg.header.stamp = rospy.Time.now()
                    self.joint_msg.position = list(result.grasp[idx].grasp_position.positions)
                    self.joint_msg.position.extend([0.0]*12)
                    self.pub.publish(self.joint_msg)
                    rospy.sleep(0.2)
                    self.joint_msg.header.stamp = rospy.Time.now()
                    self.joint_msg.position = list(result.grasp[idx].pregrasp_position.positions)
                    self.joint_msg.position.extend([0.0]*12)
                    self.pub.publish(self.joint_msg)
                    rospy.sleep(0.2)


        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.server.applyChanges()

    

def main():
    rospy.init_node("interactive_grasp_node")
    ref_frame = rospy.get_param('frame_id', 'bender/base_link')
    GraspTest(frame_id = ref_frame, init_position = Point(0.8, 0.4, 1.1))
    rospy.spin()

if __name__=="__main__":
    main()
