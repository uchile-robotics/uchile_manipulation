#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Giovanni Pais'
__email__ = 'giom.pais@gmail.com'

import rospy
from bender_arm_control.arm_commander import Limb
from moveit_python import MoveGroupInterface

from geometry_msgs.msg import Pose, PoseStamped, Point




#TEST para movimiento con skill en espacio de joint
def main():
    
    
    g = MoveGroupInterface("l_arm", "bender/base_link",None,False)

    joint_names=['l_shoulder_pitch_joint', 'l_shoulder_roll_joint',
      'l_shoulder_yaw_joint', 'l_elbow_pitch_joint', 'l_elbow_yaw_joint',
      'l_wrist_pitch_joint']
    
    rospy.loginfo('INICIA TEST')
    home =[0.0,0.0,0.0,0.0,0.0,0.0]
    pos_1=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.1624618768692017, -0.5723373293876648, 0.25651833415031433]
    pos_2=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.7700000000000017, -0.5723373293876648, 0.25651833415031433]
    l_arm_premanip_1=[-0.89, 0.129, -0.066, 1.674, -0.041, 1.053]
    
    

    #Movimiento en espacio de joint
    g.moveToJointPosition(joint_names,home)
    rospy.sleep(2.0)
    rospy.loginfo('POSITION: HOME')
    rospy.sleep(1.0)
    g.moveToJointPosition(joint_names,l_arm_premanip_1)
    rospy.sleep(2.0)
    rospy.loginfo('POSITION: l_arm_premanip_1')
    g.moveToJointPosition(joint_names,pos_1)
    rospy.sleep(2.0)
    rospy.loginfo('POSITION: pos_1')
    g.moveToJointPosition(joint_names,pos_2)
    rospy.sleep(2.0)
    rospy.loginfo('POSITION: pos_2')
    rospy.loginfo('TERMINA TEST')
    ################################


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()