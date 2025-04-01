#!/bin/python3

import cv2 as cv
import numpy as np
import os
from math import radians as rads
import random

import rospy
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError

import miro2 as miro
from miro2.lib import wheel_speed2cmd_vel

class Move():

    UPDATE = 0.02
    SLOW = 0.1
    SCAN_TURN = 0.03
    FAST = 0.4
    
    def __init__(self):
        rospy.sleep(2.0)
        self.img_convert = CvBridge()

        node_name = "test_movement"
        self.first_message = True
        
        topic_base = "/" + os.getenv("MIRO_ROBOT_NAME")

        # control publishers - for wheels and joints
        self.vel_pub = rospy.Publisher(topic_base+"/control/cmd_vel",
                                       TwistStamped, queue_size=0)
        self.joints_pub = rospy.Publisher(topic_base+"/control/kinematic_joints",
                                          JointState, queue_size=0)

        # debug publisher - for states and code checks
        self.debug_msg = rospy.Publisher("debug/msg", String, queue_size=10)

        rospy.init_node(node_name, anonymous=True)
        
        self.new_state = True
        self.bookmark = 0

        self.reset_head()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_hook)

        self.debug_msg.publish(f"The {node_name} node has been initialised...")
    
    def reset_head(self):
        self.joints = JointState()
        self.joints.name = ['tilt', 'lift', 'yaw', 'pitch']
        self.joints.position = [0.0, rads(34.0), 0.0, 0.0]
        tick = 0

        while not rospy.core.is_shutdown():
            self.joints_pub.publish(self.joints)
            rospy.sleep(self.UPDATE)
            tick += self.UPDATE
            if tick > 1:
                break
        
    def drive(self, speed_l = 0.1, speed_r = 0.1):
        vel_cmd = TwistStamped()
        wheel_speed = [speed_l, speed_r]
        (x, theta) = wheel_speed2cmd_vel(wheel_speed)

        vel_cmd.twist.linear.x = x
        vel_cmd.twist.angular.x = theta
        self.vel_pub.publish(vel_cmd)

    def search(self):
        if self.new_state:
            print("Now searching for object...")
            self.debug_msg.publish("New state - SEARCHING")
            self.new_state = False
            self.bookmark = self.counter
        
        self.drive(self.SLOW, self.SLOW)

        if self.counter > (self.bookmark + 10):
            self.state = 2
            self.new_state = True

    def found_obj(self):
        if self.new_state:
            print("Found target object...")
            self.debug_msg.publish("New state - FOUND")
            self.new_state = False
            self.bookmark = self.counter
        
        turn = random.randint(0, 1)
        # rotating clockwise
        if turn == 0:
            self.drive(self.SLOW, -self.SLOW)
        # rotating anti-clockwise
        elif turn == 1:
            self.drive(-self.SLOW, self.SLOW)

        # object no longer in vision field
        if self.counter > (self.bookmark + 5):
            self.state = random.randint(1, 3)
            print("Can no longer see object...")
            self.debug_msg.publish("Object lost - return to default state")
            self.new_state = True

    def move_towards(self):
        # TEMP: Miro will just spin when object found - avoid collision with person for now
        if self.new_state:
            print("Facing object, let's celebrate!")
            self.debug_msg.publish("New state - FACING")
            self.new_state = False
            self.bookmark = self.counter
            
        # add code to follow person!
        self.drive(self.FAST, self.FAST)

        # spin for 10 counts before resetting to default state
        if self.counter > (self.bookmark + 10):
            self.state = 0
            self.new_state = True

    def shutdown_hook(self):
        self.debug_msg.publish("Shutdown initiated...")
        self.vel_pub.publish(TwistStamped())
        self.ctrl_c = True

    def main(self):
        self.debug_msg.publish("Main loop called...")
        self.counter = 0
        self.state = 0

        while not rospy.core.is_shutdown():
            # look for object
            if self.state == 1:
                self.search()

            # face found object
            elif self.state == 2:
                self.found_obj()

            # react to finding object
            elif self.state == 3:
                self.move_towards()

            # catch and reset
            else:
                self.state = 1

            self.counter += 1
            rospy.sleep(self.UPDATE)
    
if __name__ == "__main__":
    node = Move()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass