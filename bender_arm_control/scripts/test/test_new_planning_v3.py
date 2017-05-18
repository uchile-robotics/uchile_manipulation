#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Giovanni Pais'
__email__ = 'giom.pais@gmail.com'

import rospy
from bender_arm_control.arm_commander import Limb




def main():
    
    
    limb = Limb('l')

    print "INICIA TEST"
    home =[0.0,0.0,0.0,0.0,0.0,0.0]

    print "HOMING"

    limb.arm.move_joint(home)
    rospy.sleep(5.0)

    print "POSITION: HOME"

    rospy.sleep(2.0)

    pos1=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.1624618768692017, -0.5723373293876648, 0.25651833415031433]

    pos2=[0.373582124710083, 0.4276796877384186, 0.5884131789207458, 1.7700000000000017, -0.5723373293876648, 0.25651833415031433]

    print "pos1 START"
    limb.arm.move_joint(pos1)
    rospy.sleep(5.0)
    print "pos2 START"
    limb.arm.move_joint(pos2)
    rospy.sleep(5.0)
    print "pos2 END"
    print "TERMINA TEST"
    print "BYE BYE =)"
    ################################


if __name__ == '__main__':
    rospy.init_node('planning_tester')
    main()