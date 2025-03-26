#!/bin/python3

import rospy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import miro2 as miro
from miro2.lib import wheel_speed2cmd_vel
from miro2.lib import cmd_vel2wheel_speed
from math import sqrt, pow, pi

class Square():
    def callback(self, topic_data: Odometry):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        pos_x = position.x
        pos_y = position.y

        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

        if not self.first_message:
            self.first_message = True
    
    def __init__(self):
        node_name = "test_movement"
        self.first_message = True
        self.turn = False

        self.pub = rospy.Publisher("miro/control/cmd_vel", TwistStamped, queue_size=10)
        self.sub = rospy.Subscriber("miro/sensors/odom", Odometry, self.callback)

        self.debug_msg = rospy.Publisher("debug/msg", String, queue_size=10)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(1)

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.vel = TwistStamped()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_hook)

        print(f"The {node_name} node has been initialised...")

    def shutdown_hook(self):
        print("Shutdown initiated...")
        self.pub.publish(TwistStamped())
        self.ctrl_c = True

    def main(self):
        vel = 2
        ang = 0.39
        vel_cmd = TwistStamped()
        wheel_speed = [vel, vel]
        (x, _) = wheel_speed2cmd_vel(wheel_speed)
        (_, theta) = cmd_vel2wheel_speed(vel, ang)
        seconds = 0

        while not self.ctrl_c:

            if seconds <= 4:
                vel_cmd.twist.linear.x = x
                vel_cmd.twist.angular.x = 0.0
                self.pub.publish(vel_cmd)
                self.debug_msg.publish(f"Move forward {seconds}")
            
            elif seconds > 4 and seconds <= 8:
                vel_cmd.twist.linear.x = 0.0
                vel_cmd.twist.angular.x = ang
                self.pub.publish(vel_cmd)
                self.debug_msg.publish("Stop and turn?")

            seconds += 1
            if seconds > 8:
                seconds = 0
            self.rate.sleep()
    
if __name__ == "__main__":
    node = Square()
    try:
        node.main()
    except rospy.ROSInterruptException:
        pass