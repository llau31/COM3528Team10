#!/bin/python3
###
# Code altered from com3528_examples/kick_blue_ball.py
# Editted to detect people by face and follow them
###

import os
from math import radians
import numpy as np
import cv2

import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, Bool

import miro2 as miro
from miro2.lib import wheel_speed2cmd_vel
from miro2.lib import node_detect_face

class MiroClient:
    TICK = 0.02  # This is the update interval for the main control loop in secs
    CAM_FREQ = 1  # Number of ticks before camera gets a new frame, increase in case of network lag
    SLOW = 0.1  # Radial speed when turning on the spot (rad/s)
    FAST = 0.2  # Linear speed when following the person (m/s)
    DEBUG = False # Set to True to enable debug views of the cameras
    TRANSLATION_ONLY = False # Whether to rotate only
    IS_MIROCODE = False  # Set to True if running in MiRoCODE

    def reset_head_pose(self):
        self.kin_joints = JointState()
        self.kin_joints.name = ['tilt', 'lift', 'yaw', 'pitch']
        self.kin_joints.position = [0.0, radians(34.0), 0.0, 0.0]
        t = 0

        while not rospy.core.is_shutdown():
            self.pub_kin.publish(self.kin_joints)
            rospy.sleep(self.TICK)
            t += self.TICK
            if t > 1:
                break

    def drive(self, speed_l = 0.1, speed_r = 0.1):
        msg_cmd_vel = TwistStamped()
        wheel_speed = [speed_l, speed_r]
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        self.vel_pub.publish(msg_cmd_vel)
    
    def callback_state(self, msg):
        data = msg.data
        if data == -1:
            rospy.signal_shutdown('State controller was terminated. Shutting down...')
        self.is_active = data == 2

    def callback_caml(self, ros_image):  # Left camera
        self.callback_cam(ros_image, 0)

    def callback_camr(self, ros_image):  # Right camera
        self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, index):
        """
        Callback function executed upon image arrival
        """
        # Silently(-ish) handle corrupted JPEG frames
        try:
            # Convert compressed ROS image to raw CV image
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            # Convert from OpenCV's default BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            msg_obj = miro.msg.objects()
            msg_obj.stream_index = index
            self.face_detector.tick_camera(index, msg_obj)
            # If detected by left camera, publish to left topic
            if index == 0:
                self.pub_facel.publish(msg_obj)
            # Otherwise, publish to right topic
            else:
                self.pub_facer.publish(msg_obj)

            # Store image as class attribute for further use
            self.input_camera[index] = image
            
            # Raise the flag: A new frame is available for processing
            self.new_frame[index] = True
        except CvBridgeError as e:
            # Ignore corrupted frames
            pass

    def callback_detect_face(self, msg):
        self.face[msg.stream_index] = msg

    def detect_face(self, frame, index):
        if frame is None: # Sanity check
            return
        
        if self.DEBUG:
            cv2.imshow("camera", + str(index), frame)
            cv2.waitKey(1)

        self.new_frame[index] = False
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = self.cascade.detectMultiScale(img, scaleFactor=1.1, minNeighbors=3)
        
        face_sizes = []
        for (_, _, w, h) in faces:
            size = w * h
            face_sizes.append(size)
        largest_face = faces[np.argmax(face_sizes)]
        if largest_face is not None:
            self.pub_face.publish(True)
        return largest_face # returns x, y, w, h of largest detected face

    def look_for_face(self):
        """
        [1 of 3] Rotate MiRo if it doesn't see any faces in its current
        position, until it sees one.
        """
        if self.just_switched:  # Print once
            print("MiRo is looking for a face...")
            self.just_switched = False

        for index in range(2):  # For each camera (0 = left, 1 = right)
            # Skip if there's no new image, in case the network is choking
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            # Run the detect face procedure
            self.face[index] = self.detect_face(image, index)

        # If no faces have been detected
        if not self.face[0] and not self.face[1]:
            self.drive(self.SLOW, -self.SLOW)
        else:
            self.status_code = 2  # Switch to the second action
            self.just_switched = True

    def lock_onto_face(self, error=25):
        """
        [2 of 3] Once a face has been detected, turn MiRo to face it
        """
        if self.just_switched:  # Print once
            print("MiRo is locking on to the face")
            self.just_switched = False

        for index in range(2):  # For each camera (0 = left, 1 = right)
            # Skip if there's no new image, in case the network is choking
            if not self.new_frame[index]:
                continue
            image = self.input_camera[index]
            # Run the detect face procedure
            self.face[index] = self.detect_face(image, index)

        # If only the right camera sees the face, rotate clockwise
        if not self.face[0] and self.face[1]:
            self.drive(self.SLOW, -self.SLOW)
        # Conversely, rotate counter-clockwise
        elif self.face[0] and not self.face[1]:
            self.drive(-self.SLOW, self.SLOW)

        # Make the MiRo face the person if it's visible with both cameras
        elif self.face[0] and self.face[1]:
            error = 0.05  # 5% of image width
            # Use the normalised values
            left_x = self.face[0][0]  # should be in range [0.0, 0.5]
            right_x = self.face[1][0]  # should be in range [-0.5, 0.0]
            rotation_speed = 0.03  # Turn even slower now
            if abs(left_x) - abs(right_x) > error:
                self.drive(rotation_speed, -rotation_speed)  # turn clockwise
            elif abs(left_x) - abs(right_x) < -error:
                self.drive(-rotation_speed, rotation_speed)  # turn counter-clockwise
            else:
                # Successfully turned to face the person
                self.status_code = 3  # Switch to the third action
                self.just_switched = True
                self.bookmark = self.counter

        # Otherwise, the person is lost :-(
        else:
            self.status_code = 0  # Go back to square 1...
            print("MiRo has lost the person...")
            self.just_switched = True

    def follow(self):
        """
        [3 of 3] Once MiRO is in position, this function should drive the MiRo
        towards the person!
        """
        if self.just_switched:
            print("MiRo is following the person!")
            self.just_switched = False

        if self.counter <= self.bookmark + 2 / self.TICK and not self.TRANSLATION_ONLY:
            self.drive(self.FAST, self.FAST)

        else:
            self.status_code = 0  # Back to the default state after the kick
            self.just_switched = True

    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo, if needed
        if not self.IS_MIROCODE:
            rospy.init_node("find_person", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        
        # Initialise CV Bridge
        self.image_converter = CvBridge()
        # Initialise cascade classifiers
        self.cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
       
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
       
       # Create subscriber to detect if code state is active
        self.sub_state = rospy.Subscriber(
           '/get_state',
            Int32,
            self.callback_state
        )

        # Create new subscribers to receive camera images and detect faces
        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.sub_camr = rospy.Subscriber(
            topic_base_name + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_camr,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.sub_detectl = rospy.Subscriber(
            topic_base_name + 'core/detect_objects_l',
            miro.msg.objects,
            self.callback_detect_face
        )
        self.sub_detectr = rospy.Subscriber(
            topic_base_name + 'core/detect_objects_l',
            miro.msg.objects,
            self.callback_detect_face
        )

        # Create a new publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel",
            TwistStamped,
            queue_size=0
        )
        # Create a new publisher to move the robot head
        self.pub_kin = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints",
            JointState,
            queue_size=0
        )
        # Create new publishers for face detection
        self.pub_facel = rospy.Publisher(
            topic_base_name + 'core/detect_objects_l',
            miro.msg.objects,
            queue_size=0
        )
        self.pub_facer = rospy.Publisher(
            topic_base_name + 'core/detect_objects_r',
            miro.msg.objects,
            queue_size=0
        )
        self.pub_face = rospy.Publisher(
            '/found_face',
            Bool,
            queue_size=0
        )

        self.face_detector = node_detect_face.NodeDetectFace(self)

        # Create handle to store images
        self.input_camera = [None, None]
        # New frame notification
        self.new_frame = [False, False]
        # Create variable to store a list of face's x, y, w, h values for each camera
        self.face = [None, None]
        # Set the default frame width (gets updated on receiving an image)
        self.frame_width = 640

        # Action selector to reduce duplicate printing to the terminal
        self.just_switched = True
        # Bookmark
        self.bookmark = 0
        # Move the head to default pose
        self.reset_head_pose()

    def loop(self):
        """
        Main control loop
        """
        print("MiRo finds faces, press CTRL+C to halt...")
        # Main control loop iteration counter
        self.counter = 0
        # This switch loops through MiRo behaviours:
        # Find face, lock on to the face and follow face
        self.status_code = 0
        while not rospy.core.is_shutdown():

            # Only run if state controller allows it
            if self.is_active:

                # Step 1. Find face
                if self.status_code == 1:
                    # Every once in a while, look for a face
                    if self.counter % self.CAM_FREQ == 0:
                        self.look_for_face()

                # Step 2. Orient towards it
                elif self.status_code == 2:
                    self.lock_onto_face()

                # Step 3. Follow (move towards)
                elif self.status_code == 3:
                    self.follow()

                # Fall back
                else:
                    self.status_code = 1

                # Yield
                self.counter += 1
                rospy.sleep(self.TICK)


# This condition fires when the script is called directly
if __name__ == "__main__":
    main = MiroClient()  # Instantiate class
    main.loop()  # Run the main control loop
