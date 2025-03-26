#!/bin/python3

import rospy
from std_msgs.msg import String

class Publisher():
    def __init__(self):
        self.node_name = "test_pub"

        self.pub = rospy.Publisher("test_publish", String, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo(f"The '{self.node_name}' node is active...")
        print("Node activated")

    def shutdown_hook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main(self):
        while not self.ctrl_c:
            print("New time entry...")
            msg = f"rospy time is: {rospy.get_time()}"
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    node = Publisher()
    node.main()