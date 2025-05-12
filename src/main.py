#!/usr/bin/env python3

import rospy
import os

from sensor_msgs.msg import Range

import miro2 as miro

class MiroClient:

    TICK = 0.02

    def callback_sensor(self, msg: Range):
        print(f'Distance: {msg.range} meters')

    def __init__(self):

        # Initialise node
        rospy.init_node("main", anonymous=True)
        rospy.sleep(2.0)

        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # Set up publishers/subscribers
        self.sub_sonar = rospy.Subscriber(
            topic_base_name + "/sensors/sonar",
            Range,
            self.callback_sensor
        )

        # Set up attributes
        self.just_switched = True
        self.state = 0 # States: 0=roaming, 1=wall following, 2=user following

    def loop(self):
        """
        Main control loop
        """
        print("Starting up...")

        while not rospy.core.is_shutdown():
            if self.state == 0:
                print("Roaming...")
            elif self.state == 1:
                print("Wall following...")
            elif self.state == 2:
                print("User following...")
            else:
                print("Unknown action!")
            
            rospy.sleep(self.TICK)

if __name__ == "__main__":
    client = MiroClient()
    client.loop()