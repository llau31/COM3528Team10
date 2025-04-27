#!/usr/bin/env python3

import rospy
import random
import math
import time
from geometry_msgs.msg import TwistStamped

# === Parameters ===
ALPHA = 1.5                     # Lévy alpha (Pareto tail index)
SCALE = 0.4                     # Base step size
KAPPA = 4.0                     # Concentration for Von Mises (turning)
PAUSE_DURATION = (0.5, 2.0)     # Range of pause durations
LINEAR_SPEED = 0.2              # Constant forward speed
ANGULAR_SPEED = 1.0             # Max turning speed
LOOP_HZ = 10                    # ROS loop rate

# === Helper Functions ===
def levy_step(scale=SCALE, alpha=ALPHA):
    """Sample step length using Pareto-based Lévy distribution."""
    return scale * (random.paretovariate(alpha))

def von_mises_angle(kappa=KAPPA):
    """Sample a turning angle from a Von Mises distribution (circular normal)."""
    return random.vonmisesvariate(mu=0, kappa=kappa)

def publish_twist(pub, linear, angular, duration):
    """Publish a velocity command for a fixed duration."""
    rate = rospy.Rate(LOOP_HZ)
    end_time = rospy.Time.now() + rospy.Duration(duration)
    msg = TwistStamped()
    msg.header.frame_id = "miro_link"

    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        pub.publish(msg)
        rate.sleep()

def pause():
    """Pause between movements."""
    duration = random.uniform(*PAUSE_DURATION)
    rospy.loginfo(f"🛑 Pausing for {duration:.1f}s...")
    time.sleep(duration)

def detect_obstacle():
    """
    Simulate obstacle detection (replace this with real sensor checks).
    """
    return random.random() < 0.05  # 5% chance to simulate an obstacle

# === Main Control Loop ===
def run_levy_walk():
    rospy.init_node("levy_walk_expert_node")
    pub = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=10)
    rospy.sleep(1.0)

    rospy.loginfo("🧭 Starting advanced Lévy walk for MiRo...")

    while not rospy.is_shutdown():
        # Step 1: Turn using Von Mises
        turn_angle = von_mises_angle()
        turn_duration = abs(turn_angle) / ANGULAR_SPEED
        rospy.loginfo(f"↪️ Turning {math.degrees(turn_angle):.1f}°")
        publish_twist(pub, 0.0, math.copysign(ANGULAR_SPEED, turn_angle), turn_duration)

        # Step 2: Move forward using Lévy-distributed step
        distance = levy_step()
        move_duration = distance / LINEAR_SPEED
        rospy.loginfo(f"🚶 Moving forward {distance:.2f}m")
        for _ in range(int(move_duration * LOOP_HZ)):
            if detect_obstacle():
                rospy.logwarn("🧱 Obstacle detected! Stopping early.")
                break
            publish_twist(pub, LINEAR_SPEED, 0.0, 1.0 / LOOP_HZ)

        # Step 3: Pause
        pause()

# === Run ===
if __name__ == "__main__":
    try:
        run_levy_walk()
    except rospy.ROSInterruptException:
        pass