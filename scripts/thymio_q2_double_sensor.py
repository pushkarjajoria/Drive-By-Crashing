#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
import numpy as np
import pdb
from math import pow, atan2, sqrt




class ThymioController:

    def __init__(self):
        """Initialization."""
        self.left_sensor = 0
        self.center_left_sensor = 0
        self.center_sensor = 0
        self.center_right_sensor = 0
        self.right_sensor = 0
        self.rear_left_sensor = 0
        self.rear_right_sensor = 0

        self.threshold = 0.08

        self.object_avoidance = False # Flag to stop the robot to avoid obstacles

        # initialize the node
        rospy.init_node(
        'thymio_controller'  # name of the node
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

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

        # initialize linear and angular velocities to 0
        self.velocity = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)

    def linear_velocity_controller(self, vX,vY, previous_error, proportional_gain=1.5, derivative_gain=1):
        current_error = self.euclidean_distance(vX,vY)
        error_derivative = current_error - previous_error
        linear_velocity = proportional_gain*current_error + derivative_gain*error_derivative
        linear_velocity = min(linear_velocity,.3)
        return (linear_velocity, current_error)

    def steering_angle(self, vX,vY):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        vCurr = self.human_readable_pose2d(self.pose)
        return atan2(vY - vCurr[1], vX - vCurr[0])
   
    def go_to_target(self, vX,vY):
        vel_msg = Twist()
        vCurr = self.human_readable_pose2d(self.pose)

        previous_distance_error = self.euclidean_distance(vX,vY)
        previous_angle_error = self.steering_angle(vX,vY) - vCurr[2]
        max_error_value = np.pi
        min_error_value = -np.pi
        if(previous_angle_error<min_error_value):
            previous_angle_error = previous_angle_error + 2*max_error_value
        elif(previous_angle_error>max_error_value):
            previous_angle_error = previous_angle_error - 2*max_error_value
        while self.euclidean_distance(vX,vY) >= self.distance_tolerance:
            # PD controller.
            # Linear velocity in the x-axis.
            vel_msg.linear.x, previous_distance_error = self.linear_velocity_controller(vX,vY, previous_distance_error)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

    def euclidean_distance(self, vX,vY):
        """Euclidean distance between current pose and the goal."""
        vcurr = self.human_readable_pose2d(self.pose)
        return sqrt(pow((vX - vcurr[0]), 2) +
                    pow((vY - vcurr[1]), 2))


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
        """Subscriber for robot right sensor data"""
        self.right_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True

    def log_sensor_center_right(self, data):
        """Subscriber for robot center right sensor data"""
        self.center_right_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True

    def log_sensor_center(self, data):
        """Subscriber for robot center sensor data"""
        self.center_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True

    def log_sensor_center_left(self, data):
        """Subscriber for robot center left sensor data"""
        self.center_left_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True

    def log_sensor_left(self, data):
        """Subscriber for robot left sensor data"""
        self.left_sensor = data.range
        if (data.range < self.threshold):
            self.object_avoidance = True

    def log_sensor_r_left(self, data):
        self.rear_left_sensor = data.range

    def log_sensor_r_right(self, data):
        self.rear_right_sensor = data.range


    # Checking if our movement is CW or CCW
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


    def rotate_rev(self):

        vel_msg = Twist()
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.5


        diff_threshold = 0.01
        while(abs(self.rear_right_sensor - self.rear_left_sensor) > diff_threshold or self.rear_right_sensor > 0.10):
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        return


    def rotate(self):
        def get_angular_velocity(clockwise, velocity):
            if clockwise==0:
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


        diff_threshold = 0.01
        while(abs(self.left_sensor - self.right_sensor) > diff_threshold):
            vel_msg.angular.z = get_angular_velocity(self.rotn_dirn(), velocity)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        return


    def get_control(self):
        return Twist(
        linear=Vector3(
        0.3,  # moves forward .2 m/s
        .0,
        .0,
        ),
        angular=Vector3(
        .0,
        .0,
        0.0
        )
        )

    def run(self):
        """Controls the Thymio."""
        while not rospy.is_shutdown():
        # Set a subscriber to all the proximity sensors
        # Keep Move forward
        # Stop moving forward if any sensor below threshold
        # Calculate direction of rotation based on other sensor values
        # Rotate inplace untill sensor.left and sensor.right have the same values
            while not self.object_avoidance:
                # decide control action
                velocity = self.get_control()
                # publish velocity message
                self.velocity_publisher.publish(velocity)

                # sleep until next step
                self.rate.sleep()

            self.stop()
 
            self.rotate()
            self.rotate_rev()
            curr_state = self.human_readable_pose2d(self.pose)
            print("Now moving 2m distance in the current direction")
            vDis=2
            vX_togo = curr_state[0] + 2*np.cos(curr_state[2])
            vY_togo = curr_state[1] + 2*np.sin(curr_state[2])            
            self.go_to_target(vX_togo,vY_togo)
            vDis = self.euclidean_distance(curr_state[0],curr_state[1])
            print("Reached Destination.Distance from initial location is {}".format(vDis))
            rospy.spin()

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
