#!/usr/bin/env python3

import rospy
import random
import math
import time
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range  # For ultrasound distance

# === Parameters ===
STEP_LENGTH = 1.3               # Fixed forward step length
LINEAR_SPEED = 0.5              # Constant forward speed
ANGULAR_SPEED = 1.0             # Turning speed (rad/s)
PAUSE_DURATION = (0.5, 2.0)     # Pause between steps
LOOP_HZ = 10                    # ROS loop rate
OBSTACLE_THRESHOLD = 0.5        # Stop if obstacle is closer than this (in meters)

# === Global Sensor State ===
obstacle_distance = float('inf')  # Default: no obstacle detected

# === Sensor Callback ===
def ultrasound_callback(msg):
    global obstacle_distance
    obstacle_distance = msg.range  # Update current obstacle distance in meters

# === Helper Functions ===
def random_turn_angle():
    """Generate a random turning angle between -π and π."""
    return random.uniform(-math.pi, math.pi)

def publish_twist(pub, linear, angular, duration):
    """Send a velocity command for a fixed duration."""
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
    """Pause between steps."""
    duration = random.uniform(*PAUSE_DURATION)
    rospy.loginfo(f"🛑 Pausing for {duration:.1f}s...")
    time.sleep(duration)

def detect_obstacle():
    """Check if there's an obstacle closer than the threshold."""
    return obstacle_distance < OBSTACLE_THRESHOLD

# === Main Control Loop ===
def run_random_walk():
    global obstacle_distance

    rospy.init_node("random_walk_node")
    pub = rospy.Publisher("/miro/control/cmd_vel", TwistStamped, queue_size=10)

    # Subscribe to ultrasound sensor
    rospy.Subscriber("/miro/sensors/ultrasound", Range, ultrasound_callback)

    rospy.sleep(1.0)
    rospy.loginfo("🎲 Starting random walk for MiRo with real obstacle detection...")

    while not rospy.is_shutdown():
        # Turn in a random direction
        angle = random_turn_angle()
        turn_duration = abs(angle) / ANGULAR_SPEED
        rospy.loginfo(f"↪️ Turning {math.degrees(angle):.1f}°")
        publish_twist(pub, 0.0, math.copysign(ANGULAR_SPEED, angle), turn_duration)

        # Move forward by fixed step, checking for obstacles
        move_duration = STEP_LENGTH / LINEAR_SPEED
        rospy.loginfo(f"🚶 Moving forward {STEP_LENGTH:.2f}m")
        for _ in range(int(move_duration * LOOP_HZ)):
            if detect_obstacle():
                rospy.logwarn(f"🧱 Obstacle detected at {obstacle_distance:.2f}m! Stopping early.")
                break
            publish_twist(pub, LINEAR_SPEED, 0.0, 1.0 / LOOP_HZ)

        # Optional: Log distance every second
        rospy.loginfo_throttle(1.0, f"📏 Obstacle distance: {obstacle_distance:.2f}m")

        # Pause
        pause()

# === Run Node ===
if __name__ == "__main__":
    try:
        run_random_walk()
    except rospy.ROSInterruptException:
        pass