#!/usr/bin/env python3

import rospy
import os

from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32

from roaming.random_walk import run_random_walk
from roaming.levy_walk import run_levy_walk

import miro2 as miro

class StateController:
    """
    This class controls the state of MIRO by broadcasting state changes
    to a topic.
    """

    TICK = 0.02

    # def callback_sensor(self, msg: Range):
    #     print(f'Distance: {msg.range} meters')

    def __init__(self):

        # Initialise node
        self.node_name = 'state_controller'
        topic_name = 'team10'

        # Publish to network
        self.pub = rospy.Publisher(topic_name, Int32, queue_size=10)
        rospy.init_node("state_controller", anonymous=True)
        rospy.sleep(2.0)

        # Shutdown
        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

        # Set up attributes
        self.just_switched = True
        self.state = 0 # States: 0=roaming, 1=wall following, 2=user following

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def loop(self):
        """
        Main control loop
        """
        counter = 0
        while not self.ctrl_c:
            self.pub.publish(counter)
            counter += 1
            rospy.sleep(self.TICK)

        # print("Starting up...")

        # counter = 0

        # while not rospy.core.is_shutdown():

        #     if counter > 3:
        #         self.state = 1
        #         if not self.just_switched:
        #             self.just_switched = True

        #     if self.state == 0:
        #         if self.just_switched:
        #             print("Starting levy walk...")
        #             self.just_switched = False
        #         run_levy_walk(self.pub)

        #     elif self.state == 1:
        #         if self.just_switched:
        #             print("Starting random walk...")
        #             self.just_switched = False
        #         run_random_walk(self.random_pub)

        #     elif self.state == 2:
        #         print("User following...")

        #     else:
        #         print("Unknown action!")
            
        #     print(counter)
        #     counter += 1
        #     rospy.sleep(self.TICK)

if __name__ == "__main__":
    client = StateController()
    client.loop()