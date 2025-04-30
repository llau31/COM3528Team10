#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from math import pi
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import time
import math
import tf

class FollowWallSonar():

    def sonar_callback(self, sonar_data: Range):
        self.front_range = sonar_data.range
        current_time = time.time()
        if current_time - self.start_time_2 > 1.8:
            print(f"sonar reading: {self.front_range} m")
            self.start_time_2 = current_time


    def imu_callback(self, imu_data: Imu):
        x = imu_data.orientation.x
        y = imu_data.orientation.y
        z = imu_data.orientation.z
        w = imu_data.orientation.w
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([x, y, z, w])

        current_time = time.time()
        if current_time - self.start_time > 1.8:
            #print(f"Degrees: {math.degrees(yaw)}")
            self.start_time = current_time

        
    def joints_callback(self, joints_data: JointState):
        pass




    def __init__(self):
        node_name = "follow_wall_sonar"
        rospy.init_node(node_name, anonymous=True)

        self.pub = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=10)
        self.move_head_pub = rospy.Publisher("/miro/sensors/kinematic_joints", JointState, queue_size=10)
        self.head_move_sub = rospy.Subscriber('/miro/sensors/kinematic_joints', JointState, self.joints_callback)
        self.sonar = rospy.Subscriber('/miro/sensors/sonar', Range, self.sonar_callback)
        self.imu = rospy.Subscriber('/miro/sensors/imu_body', Imu, self.imu_callback)
        
        self.move = TwistStamped()
        self.move.twist.linear.x = 0.0
        self.move.twist.angular.z = 0.0
        self.front_range = 0.0 
        self.start_time = time.time()
        self.start_time_2 = time.time()

        self.tilt_up = JointState()
        self.tilt_up.effort = [0.1]
        self.move_head_pub.publish(self.tilt_up)
        
        self.rate = rospy.Rate(10)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        self.pub.publish(TwistStamped())
        self.ctrl_c = True

    def turn(self, angle):

        angular_speed = 0.5
        angle_rad = math.radians(angle)
        turn_time = (angle_rad / angular_speed) * 2.3
        start_time = time.time()
        while time.time() - start_time < turn_time and not self.ctrl_c and not rospy.is_shutdown():
            self.move = TwistStamped()
            self.move.twist.angular.z = angular_speed
            self.pub.publish(self.move)
            self.rate.sleep()
        self.move = TwistStamped()
        self.move.twist.angular.z = 0.0
        self.pub.publish(self.move)
        print("turned")

    def main(self):
        while not rospy.is_shutdown() and not self.ctrl_c:


            self.move_head_pub.publish(self.tilt_up)
            if self.front_range > 0.12:
                #print("Not near wall")
                self.move = TwistStamped()
                #self.move.twist.linear.x = 0.1
            else:
                print("Near wall")
                self.move = TwistStamped()
                self.move.twist.linear.x = 0.0
                self.turn(90)
            self.pub.publish(self.move)

            self.rate.sleep()

if __name__ == "__main__":
    node = FollowWallSonar()
    node.main()