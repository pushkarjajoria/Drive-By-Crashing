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
        self.left_sensor = 0
        self.center_left_sensor = 0
        self.center_sensor = 0
        self.center_right_sensor = 0
        self.right_sensor = 0
        self.rear_left_sensor = 0
        self.rear_right_sensor = 0

        # Stopping threshold
        self.threshold = 0.08

        self.object_avoidance = False # Flag to stop the robot to avoid obstacles

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
        self.proximity_subscriber_r_right = rospy.Subscriber(
        self.name + '/proximity/rear_right',
        Range,
        self.log_sensor_r_right
        )
        self.proximity_subscriber_r_left = rospy.Subscriber(
        self.name + '/proximity/rear_left',
        Range,
        self.log_sensor_r_left
        )

        self.image_subscriber = rospy.Subscriber(
            self.name + '/camera/image_raw',
            numpy_msg(Image),
            self.process_image
        )
        self.image_save_frequency = 1 # seconds
        self.batch_id = str(uuid.uuid4())
        self.image_count = 0
        self.image = []


        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # initialiself.first_runze pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

        # initialize linear and angular velocities to 0
        self.velocity = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)

    def human_readable_pose2d(self, pose):
        """Converts pose message to a human readable pose tuple."""

        # create a quaternion from the pose
        quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
        )

        # convert quaternion rotation to euler rotation
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        result = (
        pose.position.x,  # x position
        pose.position.y,  # y position
        yaw  # theta angle
        )

        return result

    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = data.pose.pose
        self.velocity = data.twist.twist

        printable_pose = self.human_readable_pose2d(self.pose)

        # log robot's pose
        rospy.loginfo_throttle(
        period=5,  # log every 10 seconds
        msg=self.name + ' (%.3f, %.3f, %.3f) ' % printable_pose  # message
        )

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

    #   """Subscriber callback for rear sensors"""
    def log_sensor_r_left(self, data):
        self.rear_left_sensor = data.range

    def log_sensor_r_right(self, data):
        self.rear_right_sensor = data.range

    def create_directory(self):
        if path.exists("dataset/" + self.batch_id):
            return
        else:
            os.mkdir("dataset/" + self.batch_id)

    def save_image(self):
        image_stored = 0
        while image_stored<=1000:
            image_stored += 1           
            im = self.image
            if(len(im) == 0):
                print("No Image Data Yet, Waiting...")
                time.sleep(self.image_save_frequency)
                continue
            sensor_values = [self.left_sensor, 
                            self.center_left_sensor, 
                            self.center_sensor, 
                            self.center_right_sensor, 
                            self.right_sensor]
            np_dictinary = {"image":im,"sensor":sensor_values}
            self.image_count += 1
            np.save("dataset/" + self.batch_id + "/data_" + str(self.image_count) + ".npy", np_dictinary)
            time.sleep(self.image_save_frequency)
        os.exit(1)


    def process_image(self, data):
        self.image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        print(self.image.dtype)
        print(self.image.shape)
        print(type(self.image))

    # Calculate the direction of rotation
    def rotn_dirn(self):
        vList = [self.left_sensor,self.center_left_sensor,self.center_sensor,self.center_right_sensor,self.right_sensor]
        vInd = np.argmin(vList)
        if vInd<2:
        # rotate
          return 0
        elif vInd>2:
        # rotate
          return 1
        elif vInd==2:
        # no rotation reqd
          return 2

    # Rotating to a pose orthogonal to the obstacle and facing away
    def rotate_rev(self, randomness = False):

        vel_msg = Twist()
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.5


        diff_threshold = 0.01
        if randomness:
            diff_threshold = 0.05
        while(abs(self.rear_right_sensor - self.rear_left_sensor) > diff_threshold or self.rear_right_sensor > 0.10):
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        return



    # Rotating orthogonal to the wall and facing the wall
    def rotate(self, randomness = False):
        def get_angular_velocity(clockwise, velocity):
            if randomness:
                if random.randint(0,1):
                    return abs(velocity)
                else:
                    return -abs(velocity)
            elif clockwise==0:
                return abs(velocity)
            elif clockwise==1:
                return -abs(velocity)
            elif clockwise==2:
                return 0

        vel_msg = Twist()
        speed = 1
        velocity = 0.4
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW

        diff_threshold = 0.01
        if randomness:
            diff_threshold = 0.06

        while(abs(self.left_sensor - self.right_sensor) > diff_threshold):
            vel_msg.angular.z = get_angular_velocity(self.rotn_dirn(), velocity)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        return

    # Move slightly back from the current pose to avoid obstacle
    def reverse(self, iterations = 2):
        vel_msg = Twist()
        vel_msg.linear.x=-0.3
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        reverse_iter = 0

        while(reverse_iter < iterations):
            reverse_iter +=1 
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        return

    def robot_theta_to_absolute_theta(self, theta): # [-pi,pi] -->  [0, 2-pi]
        if theta < 0:
            return (theta + 2*np.pi)
        else:
            return theta

    def absolute_theta_to_robot_theta(self, theta): # [0, 2-pi] --> [-pi,pi]
        if theta > np.pi:
            return -(2*np.pi - theta)
        else:
            return theta

    def rotate_to_goal(self, theta, randomness = False):
        theta_tolerance = 0.2
        if randomness: # Add randomness
            theta = theta + random.uniform(-np.pi/4, np.pi/4)
        final_pose = self.absolute_theta_to_robot_theta((
            self.robot_theta_to_absolute_theta(self.human_readable_pose2d(self.pose)[2]) + theta)%(2*np.pi))
        vel_msg = Twist()
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.5

        while(abs(self.human_readable_pose2d(self.pose)[2] - final_pose) > theta_tolerance):
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        return


    def get_control(self, randomness = False):
        # to put a higher limit on total velocity
        angular_z = 0
        if randomness:
            if random.randint(0,1):
                angular_z = 0.3
            else:
                angular_z = -0.3
        return Twist(
        linear=Vector3(
        .25,  # moves forward .2 m/s
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
        self.create_directory()
        thread.start_new_thread(self.save_image, ())

        """Controls the Thymio."""
        while not rospy.is_shutdown():
        # Set a subscriber to all the proximity sensors
        # Keep Move forward
        # Stop moving forward if any sensor below threshold
        # Take a little step back
        # Rotate to a random angle behind the obstacle
        # keep doing this as part of the exploration
            while not self.object_avoidance:
                # decide control action
                velocity = self.get_control(randomness = False)
                # publish velocity message
                self.velocity_publisher.publish(velocity)

                # sleep until next step
                self.rate.sleep()

            self.stop() # Stop the robot
            self.reverse() # Move the robot a little back
            self.rotate_to_goal(np.pi, randomness=True) # Rotate to goal theta with randomness in between
            self.object_avoidance = False # Set the flag back
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
