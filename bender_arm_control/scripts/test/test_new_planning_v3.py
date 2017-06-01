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
    
    
    g = MoveGroupInterface("l_arm", "bender/base_link",None,False)  #"bender/l_shoulder_pitch_link"

    joint_names=['l_shoulder_pitch_joint', 'l_shoulder_roll_joint',
      'l_shoulder_yaw_joint', 'l_elbow_pitch_joint', 'l_elbow_yaw_joint',
      'l_wrist_pitch_joint']
    
    print "INICIA TEST"
    home =[0.0,0.0,0.0,0.0,0.0,0.0]
    pos1=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.1624618768692017, -0.5723373293876648, 0.25651833415031433]
    pos2=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.7700000000000017, -0.5723373293876648, 0.25651833415031433]
    
    #----------------------------------------------------------------
    #Datos sacados de Rviz
    #----------------------------------------------------------------
    #position= (-0.00138, 0.27986, 0.59583)
    #orientatio= (-0.014841, -0.7338, -0.0404, 0.678)
    #----------------------------------------------------------------
    #position= (0.2898, 0.2145, 0.7127)
    #orientatio= (-0.2006, -0.6747, -0.2445, 0.6667)
    #----------------------------------------------------------------

    spos=PoseStamped()
    #Position 
    #Corresponde a la transformacion de pos1 (0.60, 0.25, 0,75) otro intento (0.57, 0.2, -0.55)
    spos.pose.position.x=-0.00138
    spos.pose.position.y=0.28
    spos.pose.position.z=0.59

    #simple_orientation = Quaternion(0.0,-0.70710678,0.0,0.70710678)
    #Orientation
    spos.pose.orientation.x=-0.0148
    spos.pose.orientation.y=-0.733
    spos.pose.orientation.z=-0.0404
    spos.pose.orientation.w=0.678

    spos.header.frame_id="bender/base_link"
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