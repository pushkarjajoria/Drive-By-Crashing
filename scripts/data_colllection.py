#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Image
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
import numpy as np
import pdb
from math import pow, atan2, sqrt
import random
import time
from rospy.numpy_msg import numpy_msg
import time
import uuid
import os.path
from os import path
import thread
import os


class ThymioController:

    def __init__(self):
        """Initialization."""

        # Variable to store the current range of sensors 
        # Initializing to 10 to avoid interference with collision detection  
        self.left_sensor = 10
        self.center_left_sensor = 10
        self.center_sensor = 10
        self.center_right_sensor = 10
        self.right_sensor = 10

        # Stopping threshold
        self.threshold = 0.08

        self.collided = False # Flag to stop the robot to avoid obstacles

        # initialize the node
        rospy.init_node(
        'thymio_controller', # name of the node
        anonymous = True  
        )

        self.name = rospy.get_param('~robot_name')

        self.distance_tolerance=.1

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(
        self.name + '/cmd_vel',  # name of the topic
        Twist,  # message type
        queue_size=10  # queue size
        )

        # create pose subscriber
        self.pose_subscriber = rospy.Subscriber(
        self.name + '/odom',  # name of the topic
        Odometry,  # message type
        self.log_odometry  # function that hanldes incoming messages
        )

        ############################################################
        # Sensor Subscribers
        self.proximity_subscriber_right = rospy.Subscriber(
        self.name + '/proximity/right',
        Range,
        self.log_sensor_right
        )
        self.proximity_subscriber_center_right = rospy.Subscriber(
        self.name + '/proximity/center_right',
        Range,
        self.log_sensor_center_right
        )
        self.proximity_subscriber_center = rospy.Subscriber(
        self.name + '/proximity/center',
        Range,
        self.log_sensor_center
        )
        self.proximity_subscriber_center_left = rospy.Subscriber(
        self.name + '/proximity/center_left',
        Range,
        self.log_sensor_center_left
        )
        self.proximity_subscriber_left = rospy.Subscriber(
        self.name + '/proximity/left',
        Range,
        self.log_sensor_left
        )

        self.image_subscriber = rospy.Subscriber(
            self.name + '/camera/image_raw',
            numpy_msg(Image),
            self.process_image
        )

        self.image_save_frequency = 1 # seconds
        self.image_count = 0
        self.image = []
        self.dataset_images = []
        self.dataset_labels = []


        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # initialiself.first_runze pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

        # initialize linear and angular velocities to 0
        self.velocity = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)


    def log_sensor_right(self, data):
        """Subscriber callback for robot right sensor data"""
        self.right_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True
            self.STATE="HIT"

    def log_sensor_center_right(self, data):
        """Subscriber callback for robot right sensor data"""
        self.center_right_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True
            self.STATE="HIT"

    def log_sensor_center(self, data):
        """Subscriber callback for robot right sensor data"""
        self.center_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True
            self.STATE="HIT"


    def log_sensor_center_left(self, data):
        """Subscriber callback for robot right sensor data"""
        self.center_left_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True
            self.STATE="HIT"

    def log_sensor_left(self, data):
        """Subscriber callback for robot right sensor data"""
        self.left_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True
            self.STATE="HIT"


    def create_dataset(self):
        np.save("dataset/" + "dataset_images" + ".npy", self.dataset_images)
        np.save("dataset/" + "dataset_labels" + ".npy", self.dataset_labels)
        rospy.signal_shutdown("Data collection successfully completed!")

    def save_image(self):
        while self.image_count<=4000:
            im = self.image
            desired_size = (200,200)
            current_image = cv2.resize(im, dsize=desired_size, interpolation=cv2.INTER_CUBIC)
            
            # Saving one single image for testing purpose
            if self.image_count == 10:
                cv2.imwrite('dataset/test_img.jpg', current_image)
            
            if(len(im) == 0):
                print("No Image Data Yet, Waiting...")
                time.sleep(self.image_save_frequency)
                continue
            sensor_values = [self.left_sensor, 
                            self.center_left_sensor, 
                            self.center_sensor, 
                            self.center_right_sensor, 
                            self.right_sensor]
            self.dataset_images.append(current_image)
            self.dataset_labels.append(sensor_values)
            self.image_count += 1
            time.sleep(self.image_save_frequency)
        self.create_dataset()


    def process_image(self, data):
        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.image = np.array(im, dtype=np.float32)

    def get_control(self):
        angular_z = 0
        return Twist(
        linear=Vector3(
        .25,  # moves forward .25 m/s
        0,
        .0,
        ),
        angular=Vector3(
        .0,
        .0,
        angular_z
        )
        )

    def run(self):
        thread.start_new_thread(self.save_image, ())

        """Controls the Thymio."""
        while not rospy.is_shutdown():
        # Set a subscriber to all the proximity sensors
        # Keep Move forward
        # Stop moving forward if any sensor below threshold
        # Take a little step back
        # Rotate to a random angle behind the obstacle
        # keep doing this as part of the exploration
            while not self.collided:
                # decide control action
                velocity = self.get_control(randomness = False)
                # publish velocity message
                self.velocity_publisher.publish(velocity)

                # sleep until next step
                self.rate.sleep()

            self.collided = False # Set the flag back
        return

    def stop(self):
        """Stops the robot."""

        self.velocity_publisher.publish(
        Twist()  # set velocities to 0
        )

        self.rate.sleep()


if __name__ == '__main__':
    controller = ThymioController()

    try:
        time.sleep(15) # Added a short sleep for time to open the simulation
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
