#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import math
import tf
from std_msgs.msg import Int32

class FollowWallSonar():

    def state_callback(self, msg: Int32):
        data = msg.data
        if data == -1:
            rospy.signal_shutdown("State controller was terminated. Shutting down...")
        self.is_active = data == 1

    # uses sonar to detect objects in front of nose
    def sonar_callback(self, sonar_data: Range):
        current_time = time.time()
        if self.region != self.prev_region:
            self.front_range = sonar_data.range
            if self.region == "BOTTOM LEFT":
                print(f"sonar reading: {self.front_range} m")
            self.printed = True
        if self.left_or_right == "LEFT":
            if self.region == "BOTTOM RIGHT":
                if current_time - self.start_time_3 > 2:
                    self.start_time_3 = current_time
                if self.front_range < 0.43:
                    self.wall_pub.publish(Bool(True))
                    self.move = TwistStamped()
                    self.move.twist.angular.z = -0.75
                    self.pub.publish(self.move)
                    if current_time - self.start_time_2 > 2:
                        print("away from wall")
                        self.start_time_2 = current_time
                elif self.front_range > 0.63:
                    self.move = TwistStamped()
                    self.move.twist.angular.z = 0.75
                    self.pub.publish(self.move)
                    if current_time - self.start_time_2 > 2:
                        print("to wall")
                        self.start_time_2 = current_time
                else:
                    self.wall_pub.publish(Bool(True))
                    self.move = TwistStamped()
                    self.move.twist.angular.z = 0
                    self.pub.publish(self.move)
                    if current_time - self.start_time_2 > 2:
                        print("straight")
                        self.start_time_2 = current_time
            elif self.region == "TOP MIDDLE":
                if current_time - self.start_time_3 > 2:
                    self.start_time_3 = current_time
                if self.front_range < 0.55:
                    self.wall_pub.publish(Bool(True))
                    self.move = TwistStamped()
                    self.move.twist.angular.z = 0
                    self.to_move = False
                    print(self.move)
                    self.pub.publish(self.move)
                    self.turn(-90)
                    self.to_move = True
        # unfinished matching code for a wall on the right side
        # elif self.left_or_right == "RIGHT":
        #     if self.region == "BOTTOM LEFT":
        #         if current_time - self.start_time_3 > 2:
        #             self.start_time_3 = current_time
        #         if self.front_range < 0.43:
        #             self.wall_pub.publish(Bool(True))
        #             self.move = TwistStamped()
        #             self.move.twist.angular.z = 0.75
        #             self.pub.publish(self.move)
        #             if current_time - self.start_time_2 > 2:
        #                 print("away from wall")
        #                 self.start_time_2 = current_time
        #         elif self.front_range > 0.63:
        #             self.move = TwistStamped()
        #             self.move.twist.angular.z = -0.75
        #             self.pub.publish(self.move)
        #             if current_time - self.start_time_2 > 2:
        #                 print("to wall")
        #                 self.start_time_2 = current_time
        #         else:
        #             self.wall_pub.publish(Bool(True))
        #             self.move = TwistStamped()
        #             self.move.twist.angular.z = 0
        #             self.pub.publish(self.move)
        #             if current_time - self.start_time_2 > 2:
        #                 print("straight")
        #                 self.start_time_2 = current_time
        #     elif self.region == "TOP MIDDLE":
        #         if current_time - self.start_time_3 > 2:
        #             self.start_time_3 = current_time
        #         if self.front_range < 0.55:
        #             self.wall_pub.publish(Bool(True))
        #             self.move = TwistStamped()
        #             self.move.twist.angular.z = 0
        #             self.to_move = False
        #             print(self.move)
        #             self.pub.publish(self.move)
        #             self.turn(-90)
        #             self.to_move = True


    def imu_callback(self, imu_data: Imu):
        x = imu_data.orientation.x
        y = imu_data.orientation.y
        z = imu_data.orientation.z
        w = imu_data.orientation.w
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([x, y, z, w])


        
    def joints_callback(self, joints_data: JointState):
        # JointState includes name, position, velocity and effort
        current_time = time.time()
        temp_joint_state = JointState()
        original_positions = list(joints_data.position)
        positions = [0.0, 0.0, 0.0, 0.0]
        positions[0] = original_positions[0]
        positions[1] = original_positions[1]
        positions[2] = 0.698 * math.cos(2.4 * current_time)
        positions[3] = 0.698 * math.sin(2.4 * current_time)

        if 0.47 < math.sin(2.4 * current_time) and math.sin(2.4 * current_time) < 0.57 and -0.88 < math.cos(2.4 * current_time) and math.cos(2.4 * current_time) < -0.78:
            self.region = "BOTTOM LEFT"
        if 0.93 < math.sin(2.4 * current_time) and math.sin(2.4 * current_time) < 1 and -0.2 < math.cos(2.4 * current_time) and math.cos(2.4 * current_time) < -0.1:
            self.region = "BOTTOM MIDDLE"
        if 0.47 < math.sin(2.4 * current_time) and math.sin(2.4 * current_time) < 0.57 and 0.78 < math.cos(2.4 * current_time) and math.cos(2.4 * current_time) < 0.88:
            self.region = "BOTTOM RIGHT"
        if -0.05 < math.sin(2.4 * current_time) and math.sin(2.4 * current_time) < 0.05 and -1 < math.cos(2.4 * current_time) and math.cos(2.4 * current_time) < -0.98:
            self.region = "TOP LEFT"
        if -1 < math.sin(2.4 * current_time) and math.sin(2.4 * current_time) < -0.93 and -0.05 < math.cos(2.4 * current_time) and math.cos(2.4 * current_time) < 0.05:
            self.region = "TOP MIDDLE"
        if -0.05 < math.sin(2.4 * current_time) and math.sin(2.4 * current_time) < 0.05 and 0.98 < math.cos(2.4 * current_time) and math.cos(2.4 * current_time) < 1:
            self.region = "TOP RIGHT"

        if self.region != self.prev_region and self.printed:
            self.prev_region = self.region
            self.printed = False

        
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

        self.move = TwistStamped()
        self.move.twist.linear.x = 0.0
        self.move.twist.angular.z = 0.0
        self.front_range = 0.0 
        self.start_time = time.time()
        self.start_time_2 = time.time()
        self.start_time_3 = time.time()

        temp_joint_state = JointState()
        temp_joint_state.name = ["tilt", "lift", "yaw", "pitch"]
        temp_joint_state.position = [0.0, 0.593412, 0.0, 0.0]
        temp_joint_state.velocity = ()
        temp_joint_state.effort = ()
        
        self.region = ""
        self.prev_region = ""
        self.printed = False
        
        self.is_active = False
        self.is_break = False
        self.to_move = True
        self.left_or_right = "LEFT"

        self.pub = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=10)
        self.move_head_pub = rospy.Publisher("/miro/control/kinematic_joints", JointState, queue_size=10)
        self.head_move_sub = rospy.Subscriber('/miro/sensors/kinematic_joints', JointState, self.joints_callback)
        self.sonar = rospy.Subscriber('/miro/sensors/sonar', Range, self.sonar_callback)
        self.imu = rospy.Subscriber('/miro/sensors/imu_body', Imu, self.imu_callback)
        self.sub_state = rospy.Subscriber('/get_state', Int32, self.state_callback)
        self.wall_pub = rospy.Publisher("/wall_detected", Bool, queue_size=10)

        self.move_head_pub.publish(temp_joint_state)
        
        self.rate = rospy.Rate(10)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)


    def shutdownhook(self):
        self.pub.publish(TwistStamped())
        self.ctrl_c = True

    def turn(self, angle):
        if angle < 0:
            angular_speed = -0.5
        else:
            angular_speed = 0.5
        angle_rad = abs(math.radians(angle))
        turn_time = (angle_rad / abs(angular_speed)) * 2.3
        print(f"turn_time = {turn_time}")
        start_time = time.time()
        while time.time() - start_time < turn_time and not self.ctrl_c and not rospy.is_shutdown():
            self.move = TwistStamped()
            self.move.twist.angular.z = angular_speed
            self.pub.publish(self.move)
        self.move = TwistStamped()
        self.move.twist.angular.z = 0.0
        self.pub.publish(self.move)
        print("turned")

    def main(self):
        while not rospy.is_shutdown() and not self.ctrl_c:
            if self.is_active:
                self.move = TwistStamped()
                if self.to_move:
                    self.move.twist.linear.x = 0.9
                else:
                    self.move.twist.linear.x = 0.0
                self.pub.publish(self.move)

                self.rate.sleep()
                if self.is_break:
                    break

if __name__ == "__main__":
    node = FollowWallSonar()
    node.main()