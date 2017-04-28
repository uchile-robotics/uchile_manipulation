#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import rospy
from bender_arm_control.arm_commander import Limb
from bender_arm_control.srv import AttachObject, AttachObjectRequest
from bender_arm_planning.srv import ManageOctomap, ManageOctomapRequest
from bender_arm_planning.msg import OctomapOptions

from geometry_msgs.msg import Pose, PoseStamped, Point
from bender_msgs.msg import CylindricalObject
from tf import transformations
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes
from shape_msgs.msg import SolidPrimitive

def get_pose(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    q = transformations.quaternion_from_euler(roll, pitch, yaw)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

def get_collision_box(pose, name, dim = [0.1,0.1,0.1]):
    obj = CollisionObject()
    obj.header.stamp = rospy.Time.now()
    obj.header.frame_id = 'bender/base_link'
    obj.id = name

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = dim

    obj.primitives.append(box)
    obj.primitive_poses.append(pose)

    obj.operation = CollisionObject.ADD
    return obj

def get_collision_cylinder(pose, name, dim = [0.22, 0.04]):
    obj = CollisionObject()
    obj.header.stamp = rospy.Time.now()
    obj.header.frame_id = 'bender/base_link'
    obj.id = name

    cylinder = SolidPrimitive()
    cylinder.type = SolidPrimitive.CYLINDER
    cylinder.dimensions = dim

    obj.primitives.append(cylinder)
    obj.primitive_poses.append(pose)

    obj.operation = CollisionObject.ADD
    return obj

def main():
    rospy.wait_for_service('/l_arm_planning/attach_object')
    rospy.ServiceProxy('/l_arm_planning/attach_object', AttachObject)
    #attach_object = rospy.ServiceProxy('/l_arm_planning/attach_object', AttachObject)

    rospy.wait_for_service('/manage_octomap')
    rospy.ServiceProxy('/manage_octomap', ManageOctomap)
    #octomap = rospy.ServiceProxy('/manage_octomap', ManageOctomap)

    
    limb = Limb('l')

    ################################
    #GIO
    #For get_pose(0.60, 0.25, 0.75) 
    #(0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.1624618768692017, -0.5723373293876648, 0.25651833415031433)

    ################################
    #limb.arm.go_home()
    print "INICIA INTERVENCION GIO"
    gio_home =[0.0,0.0,0.0,0.0,0.0,0.0]

    print "HOMING"

    limb.arm.move_joint(gio_home)
    rospy.sleep(5.0)

    print "HOME STATE"

    rospy.sleep(2.0)

    gio=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.1624618768692017, -0.5723373293876648, 0.25651833415031433]

    gio2=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.7700000000000017, -0.5723373293876648, 0.25651833415031433]

    print "GIO1 START"
    limb.arm.move_joint(gio)
    rospy.sleep(5.0)
    print "GIO2 START"
    limb.arm.move_joint(gio2)
    rospy.sleep(5.0)
    print "GIO2 END"
    print "TERMINA INTERVENCION GIO"
    print "BYE BYE =)"
    ################################


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()
    