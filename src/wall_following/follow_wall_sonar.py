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

    # uses sonar to detect objects in front of nose
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
        if current_time - self.start_time > 2:
            #print(f"Degrees: {math.degrees(yaw)}")
            self.start_time = current_time

        
    def joints_callback(self, joints_data: JointState):
        # JointState includes name, position, velocity and effort
        current_time = time.time()
        #if current_time - self.start_time_3 > 2:
            # tilt, lift, yaw, pitch
            # print(joints_data.name)
            # print(joints_data.position)
            # print(joints_data.velocity)
            # print(joints_data.effort)
            # self.start_time_3 = current_time
        current_time = time.time()
        temp_joint_state = JointState()
        original_positions = list(joints_data.position)
        positions = [0.0, 0.0, 0.0, 0.0]
        new_positions = [(-0.698, 0), (-0.698, -0.262), (0.698, -0.262), (0.698, 0)]
        # positions[0] = original_positions[0]
        # positions[1] = original_positions[1]
        # positions[2] = original_positions[2]
        # positions[2] = 0.698 * math.sin(0.5 * current_time)
        # positions[2] = 0.698 * math.cos(0.5 * current_time)
        # positions[3] = original_positions[3]
        # positions[3] = 0.698 * math.sin(0.5 * current_time)
        # positions[3] = 0.698 * math.cos(0.5 * current_time)
        self.step = 0
        # print(current_time)
        # print("current_time")
        # print(self.start_time_3)
        # print("self.start_time_3")
        # print(current_time - self.start_time_3 > 2)
        if current_time - self.start_time_3 > 3:
            positions[0] = original_positions[0]
            positions[1] = original_positions[1]
            positions[2] = new_positions[self.step][self.step]
            positions[2] = 0.698 * math.cos(0.5 * current_time)
            self.step = (self.step + 1) % 4
            self.start_time_3 = current_time
            print("Working")
            print(positions)
            print("positions")  
        temp_joint_state.position = positions
        temp_joint_state.name = ["tilt", "lift", "yaw", "pitch"]
        temp_joint_state.velocity = ()
        temp_joint_state.effort = ()
        self.move_head_pub.publish(temp_joint_state)
        self.move = TwistStamped()
        self.move.twist.angular.x = 0.0
        self.move.twist.angular.y = 0.0
        self.move.twist.angular.z = 0.0
        self.move.twist.linear.x = 0.0
        self.move.twist.linear.y = 0.0
        self.move.twist.linear.z = 0.0
        self.pub.publish(self.move)


    def __init__(self):
        node_name = "follow_wall_sonar"
        rospy.init_node(node_name, anonymous=True)

        self.pub = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=10)
        self.move_head_pub = rospy.Publisher("/miro/control/kinematic_joints", JointState, queue_size=10)
        self.head_move_sub = rospy.Subscriber('/miro/sensors/kinematic_joints', JointState, self.joints_callback)
        self.sonar = rospy.Subscriber('/miro/sensors/sonar', Range, self.sonar_callback)
        self.imu = rospy.Subscriber('/miro/sensors/imu_body', Imu, self.imu_callback)
        
        self.move = TwistStamped()
        self.move.twist.linear.x = 0.0
        self.move.twist.angular.z = 0.0
        self.front_range = 0.0 
        self.start_time = time.time()
        self.start_time_2 = time.time()
        self.start_time_3 = time.time()

        temp_joint_state = JointState()
        temp_joint_state.name = ["tilt", "lift", "yaw", "pitch"]
        temp_joint_state.position = [0.0, 0.0, 0.0, 0.0]
        temp_joint_state.velocity = ()
        temp_joint_state.effort = ()
        self.move_head_pub.publish(temp_joint_state)
        
        
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
            # if self.front_range > 0.12:
            #     #print("Not near wall")
            #     self.move = TwistStamped()
            #     self.move.twist.linear.x = 0.1
            # else:
            #     print("Near wall")
            #     self.move = TwistStamped()
            #     self.move.twist.linear.x = 0.0
            #     self.turn(90)
            # self.pub.publish(self.move)

            self.rate.sleep()

if __name__ == "__main__":
    node = FollowWallSonar()
    node.main()