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
from hb_workspace_analysis.srv import GetCapabilityMap, GetCapabilityMapRequest

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

class InteractiveGrasp(object):

    def __init__(self, topic_name = "interactive_grasp", frame_id = "base_link", arm_name = 'l_arm ',radius = 0.04, height = 0.22, init_position = Point(0,0,0)):
        # Interactive Marker
        self.menu_handler = MenuHandler()
        self.server = InteractiveMarkerServer(topic_name)
        self.current_pose = Pose()
        self.current_pose.position = init_position

        self.menu_handler.insert( "Grasp", callback=self.process_feedback)
        self.add_grasp_marker(frame_id, radius, height, init_position)
        self.server.applyChanges()
        self.radius = radius
        self.height = height
        self.frame_id = frame_id

        self.grasp_server = rospy.ServiceProxy('/capability_map', GetCapabilityMap)
        self.pub = rospy.Publisher('test/joint_states', JointState, queue_size=10)
        self.joint_msg = JointState()
        self.joint_msg.name = ['l_shoulder_pitch_joint', 'l_shoulder_roll_joint', 'l_shoulder_yaw_joint', 'l_elbow_pitch_joint', 'l_elbow_yaw_joint', 'l_wrist_pitch_joint',
              'l_ext_finger_joint'
              ,'l_int_finger_joint'
              ,'head_yaw_joint'
              ,'head_pitch_joint'
              ,'r_shoulder_pitch_joint'
              ,'r_shoulder_roll_joint'
              ,'r_shoulder_yaw_joint'
              ,'r_elbow_pitch_joint'
              ,'r_elbow_yaw_joint'
              ,'r_wrist_pitch_joint'
              ,'r_ext_finger_joint'
              ,'r_int_finger_joint']
        self.joint_msg.velocity = [0.0]*len(self.joint_msg.name)
        self.joint_msg.effort = [0.0]*len(self.joint_msg.name)

      
    def process_feedback(self, feedback):
        # Update pose
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.current_pose = feedback.pose

        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # Grasp
            if feedback.menu_entry_id == 1:
                # Get object pose
                req = GetCapabilityMapRequest()
                req.object = get_cylinder(self.current_pose)
                try:
                    result = self.grasp_server(req)
                    if result.grasp.grasp:
                        # Show at least 10 random grasp position
                        shuffle(result.grasp.grasp)
                        i = 0
                        for idx in xrange(min(10, len(result.grasp.grasp))):
                            self.joint_msg.header.stamp = rospy.Time.now()
                            self.joint_msg.position = list(result.grasp.grasp[idx].grasp_position.positions)
                            self.joint_msg.position.extend([0.0]*12)
                            self.pub.publish(self.joint_msg)
                            rospy.sleep(0.03)
                            self.joint_msg.header.stamp = rospy.Time.now()
                            self.joint_msg.position = list(result.grasp.grasp[idx].pregrasp_position.positions)
                            self.joint_msg.position.extend([0.0]*12)
                            self.pub.publish(self.joint_msg)
                            rospy.sleep(0.03)


                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
        self.server.applyChanges()

    def add_grasp_marker(self, frame_id, radius, height, init_position):
        # Basic description
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.pose.position = init_position
        int_marker.scale = max(radius*2, height) + 0.02
        int_marker.name = "grasp_marker"
        int_marker.description = "Graspable object"
        # Geometry
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.scale.x = radius*2
        marker.scale.y = radius*2
        marker.scale.z = height
        marker.color.r = random()
        marker.color.g = random()
        marker.color.b = random()
        marker.color.a = 1.0
        # Control 6DOF
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        # Control roll
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "roll"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        # Movimiento en X
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        # Control yaw
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "yaw"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        # Movimiento en Z
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        # Control pitch
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "pitch"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        # Movimiento en y
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        self.server.insert(int_marker, self.process_feedback)
        self.menu_handler.apply(self.server, int_marker.name)

def main():
    rospy.init_node("interactive_grasp_node")
    ref_frame = rospy.get_param('frame_id', 'bender/base_link')
    InteractiveGrasp(frame_id = ref_frame, init_position = Point(0.8, 0.4, 1.1))
    rospy.spin()

if __name__=="__main__":
    main()
