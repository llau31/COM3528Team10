#!/usr/bin/env python3

import rospy
import os

from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32
from std_msgs.msg import Bool

# from roaming.random_walk import run_random_walk
# from roaming.levy_walk import run_levy_walk

import miro2 as miro

class StateController:
    """
    This class controls the state of MIRO by broadcasting state changes
    to topic /get_state
    """

    TICK = 0.02

    # def callback_sensor(self, msg: Range):
    #     print(f'Distance: {msg.range} meters')
    # def wall_callback(self, wall: Bool):
    #     self.wall_detected = wall.data

    def callback_face(self, msg: Bool):
        self.face_detected = msg.data

    def __init__(self):

        # Initialise node
        self.node_name = 'state_controller'
        topic_name = 'get_state'

        # Publish to network
        self.pub = rospy.Publisher(topic_name, Int32, queue_size=10)
        self.wall_sub = rospy.Subscriber('/wall_detected', Bool, self.wall_callback)
        rospy.init_node("state_controller", anonymous=True)
        rospy.sleep(2.0)

        # Subscribers
        self.sub_face = rospy.Subscriber('/found_face', Bool, self.callback_face)

        # Shutdown
        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

        # Set up attributes
        self.state = 0 # States: -1=terminate, 0=roaming, 1=wall following, 2=user following
        self.face_detected = False
        self.wall_detected = False

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")

        # Terminate all nodes
        self.pub.publish(-1)
        self.ctrl_c = True

    def loop(self):
        """
        Main control loop
        """

        while not self.ctrl_c:

            if self.face_detected:
                self.state = 2

            elif self.wall_detected:
                self.state = 1

            else:
                self.state = 0

            self.pub.publish(self.state)
            rospy.sleep(self.TICK)

if __name__ == "__main__":
    client = StateController()
    client.loop()