#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import rospy
from bender_arm_control.arm_commander import Limb
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import Joy
from tf import transformations
from vlc_video import VLCPlayBackground, get_video_files
from threading import Lock

def getPose(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    q = transformations.quaternion_from_euler(roll, pitch, yaw)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p

class SimpleArmTeleop(object):
  
    BUTTONS = {
        # Arm
        'hold' : 4,  # LB
        'reset' : 5, # LB + RB
        'forward': 13,  # LB + UP
        'backward': 14, # LB + DOWN
        'change_arm': 6, # LB + BACK
        'open_gripper': 11, # LB + LEFT
        'close_gripper': 12, # LB + RIGHT
        'wave': 10, # LB + Right stick
        # Video
        'hold_video': 5, # RB
        'forward_video': 13,  # RB + UP
        'backward_video': 14 # RB + DOWN
    }

    VIDEO_FOLDER = '/home/bender-chest/videos'

    """"Control de brazos a posiciones predefinidas"""
    def __init__(self):

        rospy.logwarn('Init arm interface')

        arm_opc = rospy.get_param('~use_arm', 'r_arm')
        rospy.logerr(arm_opc)

        # Default selected arm
        self.selected_arm_side = 'r'
        self.arms = dict()
        if arm_opc == "both":
            rospy.loginfo('using both')
            self.arms = {
                'l':Limb('l', enable_planning=False),
                'r':Limb('r', enable_planning=False)
            }
        
        elif arm_opc == "l_arm":
            rospy.loginfo('using l arm')
            self.arms = { 'l':Limb('l', enable_planning=False)}
          
        elif arm_opc == "r_arm":
            rospy.loginfo('using r arm')
            self.arms = { 'r':Limb('r', enable_planning=False)}
            self.selected_arm_side = 'r'
        self.arm_lock = Lock()

        # Topico joystick
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.process_joy, queue_size=2)
        rospy.loginfo("ArmTeleop OK using arm:" + arm_opc)

        # State
        self.move_state = 0

        # Video playback
        self.vlc = VLCPlayBackground()
        self.video_list = get_video_files(SimpleArmTeleop.VIDEO_FOLDER, extensions=['mp4','avi'])
        self.video_len = len(self.video_list)
        self.video_selected = 0
        self.video_lock = Lock()
        rospy.loginfo('Video files: ')
        for vid in self.video_list:
            rospy.loginfo('\t{}'.format(vid))
        

    def process_joy(self, msg):
        self.process_joy_video(msg)
        self.process_joy_arm(msg)

    def process_joy_video(self, msg):
        if self.video_lock.locked():
            return
        with self.video_lock:
            # Hold button must be pressed
            if not msg.buttons[SimpleArmTeleop.BUTTONS['hold_video']] == 1:
                return
            # Check list
            if not self.video_list:
                return

            # Forward
            if msg.buttons[SimpleArmTeleop.BUTTONS['forward_video']] == 1:
                self.video_selected = (self.video_selected+1)%self.video_len
                 # Play video
                self.vlc.play(self.video_list[self.video_selected])
                rospy.sleep(1.0)

            # Backward
            if msg.buttons[SimpleArmTeleop.BUTTONS['backward_video']] == 1:
                self.video_selected = (self.video_selected-1)%self.video_len
                # Play video
                self.vlc.play(self.video_list[self.video_selected])
                rospy.sleep(1.0)

    def process_joy_arm(self, msg):
        if self.arm_lock.locked():
            return
        with self.arm_lock:
            # Hold button must be pressed
            if not msg.buttons[SimpleArmTeleop.BUTTONS['hold']] == 1:
                return

            # Reset move state
            # if msg.buttons[SimpleArmTeleop.BUTTONS['reset']] == 1:
            #     rospy.logwarn('Reset move state')
            #     self.move_state = 0

            # Check selected arm
            if not self.selected_arm_side in self.arms:
                rospy.logwarn('Selected arm: ' + self.selected_arm_side + ' not available')
                rospy.sleep(1.0)
                return
            selected_arm = self.arms[self.selected_arm_side]
            
            # Forward
            if msg.buttons[SimpleArmTeleop.BUTTONS['forward']] == 1:
                if self.move_state == 0:
                    selected_arm.arm.move_joint_blind('home', 'pre_1')
                    self.move_state = 1
                elif self.move_state == 1:
                    selected_arm.arm.move_joint_blind('pre_1', 'pre_2')
                    self.move_state = 2
                rospy.sleep(1.0)

            # Backward
            if msg.buttons[SimpleArmTeleop.BUTTONS['backward']] == 1:
                if self.move_state == 2:
                    selected_arm.arm.move_joint_blind('pre_2', 'pre_1')
                    self.move_state = 1
                elif self.move_state == 1:
                    selected_arm.arm.move_joint_blind('pre_1', 'home')
                    self.move_state = 0
                rospy.sleep(1.0)
              
            # Change arm
            elif msg.buttons[SimpleArmTeleop.BUTTONS['change_arm']] == 1:
                if self.selected_arm_side == 'l' and 'r' in self.arms:
                    self.selected_arm_side = 'r'
                elif self.selected_arm_side == 'r' and 'l' in self.arms:
                    self.selected_arm_side = 'l'
                rospy.logwarn('Selected arm: ' + self.selected_arm_side)
                rospy.sleep(1.0)

            # Wave
            # elif msg.buttons[SimpleArmTeleop.BUTTONS['wave']] == 1:
            #     rospy.loginfo('Waving')
            #     selected_arm.arm.move_joint([0.0, 0.0, 0.0, 1.45, 0.0, 1.0], interval=4.5)
            #     rospy.sleep(2.0)
            #     selected_arm.arm.move_joint([0.0, 0.0, 0.0, 1.45, 0.7, 1.0], interval=1.2)
            #     for i in xrange(5):
            #         selected_arm.arm.move_joint([0.0, 0.0, 0.0, 1.45, 0.7, 1.0], interval=0.8)
            #         rospy.sleep(0.7)
            #         selected_arm.arm.move_joint([0.0, 0.0, 0.0, 1.45, -0.7, 1.0], interval=0.8)
            #         rospy.sleep(0.7)
            #     selected_arm.arm.move_joint([0.0, 0.0, 0.0, 1.45, 0.0, 1.0], interval=2.0)

            # Open gripper
            elif msg.buttons[SimpleArmTeleop.BUTTONS['open_gripper']] == 1:
                rospy.loginfo('Gripper open')
                selected_arm.gripper.open()
                rospy.sleep(1.0)
                # raw command: gripper.command(position, effort)
              
            # Close gripper
            elif msg.buttons[SimpleArmTeleop.BUTTONS['close_gripper']] == 1:
                rospy.loginfo('Gripper close')
                selected_arm.gripper.close(effort=0.15)
                rospy.sleep(1.0)


def main():
    SimpleArmTeleop()
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('arm_teleop')
    main()

