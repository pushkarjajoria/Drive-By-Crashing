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
import cv2
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class ThymioController:

    def __init__(self):
        """Initialization."""
        self.left_sensor = 10
        self.center_left_sensor = 10
        self.center_sensor = 10
        self.center_right_sensor = 10
        self.right_sensor = 10


        # initialize the node
        rospy.init_node(
        'thymio_controller', # name of the node
        anonymous = True  
        )

        self.collision_tol = .02
        ## changes
        self.vX = np.array([1,2,5,6,7])
        self.vY = np.array([-1,-3,-5,-4])

        self.collided = False # Flag to stop the robot to avoid obstacles


        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(
        self.name + '/cmd_vel',  # name of the topic
        Twist,  # message type
        queue_size=10  # queue size
        )

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

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # set node update frequency in Hz
        self.rate = rospy.Rate(50)


    def log_sensor_right(self, data):
        """Subscriber callback for robot right sensor data"""
        self.right_sensor = data.range

    def log_sensor_center_right(self, data):
        """Subscriber callback for robot right sensor data"""
        self.center_right_sensor = data.range

    def log_sensor_center(self, data):
        """Subscriber callback for robot right sensor data"""
        self.center_sensor = data.range

    def log_sensor_center_left(self, data):
        """Subscriber callback for robot right sensor data"""
        self.center_left_sensor = data.range

    def log_sensor_left(self, data):
        """Subscriber callback for robot right sensor data"""
        self.left_sensor = data.range

    def respawn(self,theta): # theta = [-pi/2, pi/2]
        # http://wiki.ogre3d.org/Quaternion+and+Rotation+Primer
        w = np.cos(theta/2) 
        z = np.sin(theta/2)
        vMsg = ModelState()
        vMsg.model_name = self.name
        vMsg.model_name = vMsg.model_name[1:]
        x = np.random.choice(self.vX,1)
        y = np.random.choice(self.vY,1)
        vMsg.pose.position.x = float(x)
        vMsg.pose.position.y = float(y)
        vMsg.pose.position.z = 0

        vMsg.pose.orientation.x = 0
        vMsg.pose.orientation.y = 0
        vMsg.pose.orientation.z = z
        vMsg.pose.orientation.w = w
        rospy.wait_for_service('/gazebo/set_model_state')

        try:
            vState = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            return vState(vMsg)
        except rospy.ServiceException, e:
            print(e)


    def get_control(self, linear_x=0.1):
        sensor_values = [self.left_sensor, 
                        self.center_left_sensor, 
                        self.center_sensor, 
                        self.center_right_sensor, 
                        self.right_sensor]

        # to put a higher limit on total velocity
        multiplicative_factor = 20
        angular_z = multiplicative_factor*np.dot(sensor_values, [1,2,0,-2,-1])/3
        return Twist(
        linear=Vector3(
        linear_x,  # moves forward .2 m/s
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
        """Controls the Thymio."""
        while not rospy.is_shutdown():
            if self.collided==True:
                q_theta = np.random.uniform(-1,1)*np.pi
                self.respawn(q_theta)
                self.collided=False


            while not self.collided:
                # decide control action
                velocity = self.get_control()
                # publish velocity message
                self.velocity_publisher.publish(velocity)
                
                vsensor = np.array([self.left_sensor, 
                            self.center_left_sensor, 
                            self.center_sensor, 
                            self.center_right_sensor, 
                            self.right_sensor])

                if min(vsensor)<self.collision_tol:
                    #print("*"*25)
                    #print("Collision!!!!")
                    self.collided=True

                # sleep until next step
            self.rate.sleep()

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
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
