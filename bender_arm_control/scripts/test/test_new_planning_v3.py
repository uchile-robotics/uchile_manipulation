#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Giovanni Pais'
__email__ = 'giom.pais@gmail.com'

import rospy
from bender_arm_control.arm_commander import Limb
from moveit_python import MoveGroupInterface

from geometry_msgs.msg import Pose, PoseStamped, Point





def main():
    
    
    g = MoveGroupInterface("l_arm", "bender/l_shoulder_pitch_link",None,False)

    joint_names=['l_shoulder_pitch_joint', 'l_shoulder_roll_joint',
      'l_shoulder_yaw_joint', 'l_elbow_pitch_joint', 'l_elbow_yaw_joint',
      'l_wrist_pitch_joint']
    
    print "INICIA TEST"
    home =[0.0,0.0,0.0,0.0,0.0,0.0]
    pos1=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.1624618768692017, -0.5723373293876648, 0.25651833415031433]
    pos2=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.7700000000000017, -0.5723373293876648, 0.25651833415031433]
    
    spos=PoseStamped()
    #Position 
    #Corresponde a la transformacion de pos1 (0.60, 0.25, 0,75) otro intento (0.57, 0.2, -0.55)
    spos.pose.position.x=0.40
    spos.pose.position.y=0.0
    spos.pose.position.z=-0.40

    #simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)
    #Orientation
    spos.pose.orientation.x=0.0
    spos.pose.orientation.y=-0.70710678
    spos.pose.orientation.z=0.0
    spos.pose.orientation.w=0.70710678

    spos.header.frame_id="bender/l_shoulder_pitch_link"
    #/bender/base_link'


    #Movimiento en espacio de joint
    g.moveToJointPosition(joint_names,home)

    rospy.sleep(3.0)
    print "POSITION: HOME"

    #Movimiento en espacio cartesiano
    g.moveToPose(spos,"bender/l_wrist_pitch_link")

    rospy.sleep(2.0)

    

    rospy.sleep(1.0)


    
    # print "pos1 START"
    # limb.arm.move_joint(pos1)
    # rospy.sleep(5.0)
    # print "pos2 START"
    # limb.arm.move_joint(pos2)
    # rospy.sleep(5.0)
    # print "pos2 END"
    print "TERMINA TEST"
    print "BYE BYE =)"
    ################################


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()