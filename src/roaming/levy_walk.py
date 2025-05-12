#!/usr/bin/env python3

import rospy
import random
import math
import time
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32

class LevyWalk:

    # === Parameters ===
    ALPHA = 1.5                     # Lévy alpha (Pareto tail index)
    SCALE = 0.4                     # Base step size
    KAPPA = 4.0                     # Concentration for Von Mises (turning)
    PAUSE_DURATION = (0.5, 2.0)     # Range of pause durations
    LINEAR_SPEED = 0.2              # Constant forward speed
    ANGULAR_SPEED = 1.0             # Max turning speed
    LOOP_HZ = 10                    # ROS loop rate

    def state_callback(self, msg: Int32):
        data = msg.data

        if data == -1:
            rospy.signal_shutdown('State controller was terminated. Shutting down...')

        self.is_active = data == 0

    def shutdownhook(self): 
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def __init__(self):
        # Initialise node on network
        self.node_name = 'levy_walk'
        rospy.init_node(self.node_name, anonymous=True)

        # Publishers
        self.pub_cmd_vel = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=10)

        # Subscribers
        self.sub_state = rospy.Subscriber('/get_state', Int32, self.state_callback)

        rospy.sleep(2)
        rospy.loginfo("🧭 Starting advanced Lévy walk for MiRo...")

        # Initialise variables
        self.is_turning = True
        self.vel = TwistStamped()
        self.just_switched = True
        self.rate = rospy.Rate(self.LOOP_HZ)
        self.first_loop = True
        self.is_active = False
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook) 

        # Turning
        self.turn_angle = 0
        self.turn_duration = -1
        self.end_turn = rospy.Time()

        # Moving
        self.distance = 0
        self.move_duration = -1
        self.end_move = rospy.Time()

    # === Helper Functions ===
    def levy_step(self, scale=SCALE, alpha=ALPHA):
        """Sample step length using Pareto-based Lévy distribution."""
        return scale * (random.paretovariate(alpha))

    def von_mises_angle(self, kappa=KAPPA):
        """Sample a turning angle from a Von Mises distribution (circular normal)."""
        return random.vonmisesvariate(mu=0, kappa=kappa)

    def publish_twist(self, pub, linear, angular, duration):
        """Publish a velocity command for a fixed duration."""
        rate = rospy.Rate(self.LOOP_HZ)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        msg = TwistStamped()
        msg.header.frame_id = "miro_link"

        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            msg.twist.linear.x = linear
            msg.twist.angular.z = angular
            pub.publish(msg)
            rate.sleep()

    def pause(self):
        """Pause between movements."""
        duration = random.uniform(*self.PAUSE_DURATION)
        rospy.loginfo(f"🛑 Pausing for {duration:.1f}s...")
        time.sleep(duration)

    def detect_obstacle(self):
        """
        Simulate obstacle detection (replace this with real sensor checks).
        """
        return random.random() < 0.05  # 5% chance to simulate an obstacle

    # === Main Control Loop ===
    def run_levy_walk(self):
        while not rospy.is_shutdown():
            # Step 1: Turn using Von Mises
            turn_angle = self.von_mises_angle()
            turn_duration = abs(turn_angle) / self.ANGULAR_SPEED
            rospy.loginfo(f"↪️ Turning {math.degrees(turn_angle):.1f}°")
            self.publish_twist(self.pub_cmd_vel, 0.0, math.copysign(self.ANGULAR_SPEED, turn_angle), turn_duration)

            # Step 2: Move forward using Lévy-distributed step
            distance = self.levy_step()
            move_duration = distance / self.LINEAR_SPEED
            rospy.loginfo(f"🚶 Moving forward {distance:.2f}m")
            for _ in range(int(move_duration * self.LOOP_HZ)):
                if self.detect_obstacle():
                    rospy.logwarn("🧱 Obstacle detected! Stopping early.")
                    break
                self.publish_twist(self.pub_cmd_vel, self.LINEAR_SPEED, 0.0, 1.0 / self.LOOP_HZ)

            # Step 3: Pause
            self.pause()

    def control_turning(self):
        """
        Controls whether miro is turning or not.
        """
        
        t = rospy.Time.now()

        if t > self.end_turn and t > self.end_move:
            if self.end_turn > self.end_move:
                self.start_move()
                return 
            self.start_turn()
            return    
        return

    def start_turn(self):
        """
        Initiate turning.
        """

        rospy.sleep(1)

        self.turn_angle = self.von_mises_angle()
        self.turn_duration = abs(self.turn_angle) / self.ANGULAR_SPEED
        self.end_turn = rospy.Time.now() + rospy.Duration(self.turn_duration)

        rospy.loginfo(f"↪️ Turning {math.degrees(self.turn_angle):.1f}°")
        self.is_turning = True

    def start_move(self):
        """
        Initiate moving.
        """

        rospy.sleep(1)

        self.distance = self.levy_step()
        self.move_duration = self.distance / self.LINEAR_SPEED
        self.end_move = rospy.Time.now() + rospy.Duration(self.move_duration)

        rospy.loginfo(f"🚶 Moving forward {self.distance:.2f}m")
        self.is_turning = False

    def main(self):
        """
        Main loop for integration.
        """

        while not self.ctrl_c:

            # Only run if state controller allows it
            if self.is_active:
                if self.just_switched:
                    self.start_turn()
                    self.just_switched = False

                # Check if miro is turning
                self.control_turning()

                if self.is_turning:
                    self.vel.twist.linear.x = 0
                    self.vel.twist.angular.z = math.copysign(self.ANGULAR_SPEED, self.turn_angle)

                else:
                    self.vel.twist.linear.x = self.LINEAR_SPEED
                    self.vel.twist.angular.z = 0

                self.pub_cmd_vel.publish(self.vel)

            else:
                self.just_switched = True

            self.rate.sleep()

# === Run ===
if __name__ == "__main__":
    try:
        levy_walk = LevyWalk()
        levy_walk.main()
    except rospy.ROSInterruptException:
        pass