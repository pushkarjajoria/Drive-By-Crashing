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
import torch.nn as nn
import torch.nn.functional as F
import torch
import cv2


class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, 7)
        self.pool2 = nn.MaxPool2d(4, 4)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(32, 64, 5)
        self.conv3 = nn.Conv2d(64, 16, 5)
        self.fc1 = nn.Linear(448, 128)
        self.fc2 = nn.Linear(128, 10)
        self.fc3 = nn.Linear(10, 1)

    def forward(self, x):
        x = self.pool2(F.relu(self.conv1(x)))
        x = self.pool2(F.relu(self.conv2(x)))
        x = self.pool(F.relu(self.conv3(x)))
        v_to = 1
        for i in range(1,len(x.shape)):
          v_to = v_to*x.shape[i]
        x = x.view(-1, v_to)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        x = F.tanh(x)
        return x

class Cnn_model:
    def __init__(self):
        self.vFile = "/home/usi/Desktop/249_model.pt"


    def load_model(self):
        use_cuda = torch.cuda.is_available()
        DEVICE = torch.device('cuda' if use_cuda else 'cpu')   # 'cpu' in this case
        cpu_model = Net()
        cpu_model.load_state_dict(torch.load(self.vFile, map_location=DEVICE))
        return cpu_model


class ThymioController:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
        'thymio_controller', # name of the node
        anonymous = True  
        )

        self.cnn_model = Cnn_model()
        self.model = self.cnn_model.load_model()
        self.model.eval()

        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(
        self.name + '/cmd_vel',  # name of the topic
        Twist,  # message type
        queue_size=10  # queue size
        )

        self.image_subscriber = rospy.Subscriber(
            self.name + '/camera/image_raw',
            numpy_msg(Image),
            self.process_image
        )
        self.image = np.zeros((500,500,3), dtype = np.float32)


        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # set node update frequency in Hz
        self.rate = rospy.Rate(500)

    def cnn_controller(self):
        desired_size = (240,320)
        current_image = cv2.resize(self.image, dsize=desired_size, interpolation=cv2.INTER_CUBIC)
        v_image = torch.from_numpy(current_image)
        vShape = v_image.size()
        v_image = v_image.reshape((1,vShape[2],vShape[0],vShape[1]))

        ## in pytorch unlike TF image has to be passed as (batch, channels, height, width) therefore reshaping.
        self.model.eval()
        angular_z = self.model(v_image)
        return angular_z


    def process_image(self, data):
        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.image = np.array(im, dtype=np.float32)
        #print("*"*25)
        #print(np.array(self.image).dtype)
        #print(self.image.dtype)
        #print("*"*25)

    def get_control(self, linear_x=0.25):
        # to put a higher limit on total velocity
        angular_z = self.cnn_controller()
        return Twist(
        linear=Vector3(
        linear_x,  # moves forward .2 m/s
        0,
        .0,
        ),
        angular=Vector3(
        .0,
        .0,
        angular_z*0.25
        )
        )

    def run(self):
        """Controls the Thymio."""
        while not rospy.is_shutdown():
            # decide control action
            velocity = self.get_control()
            # publish velocity message
            self.velocity_publisher.publish(velocity)

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
