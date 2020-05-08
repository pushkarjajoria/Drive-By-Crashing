#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion



class ThymioController:

    def __init__(self):
		"""Initialization."""
		self.left_sensor = 0
		self.center_left_sensor = 0
		self.center_sensor = 0
		self.center_right_sensor = 0
		self.right_sensor = 0

		self.threshold = 0.05

		self.object_avoidance = False # Flag to stop the robot to avoid obstacles

		# initialize the node
		rospy.init_node(
		    'thymio_controller'  # name of the node
		)

		self.name = rospy.get_param('~robot_name')

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

#		self.proximity_subscriber_right = rospy.Subscriber(
#			self.name + '/proximity/right',
#			Range,
#			self.log_sensor_right
#		)

		self.proximity_subscriber_center_right = rospy.Subscriber(
			self.name + '/proximity/center_right',
			Range,
			self.log_sensor_center_right
		)

#		self.proximity_subscriber_center = rospy.Subscriber(
#			self.name + '/proximity/center',
#			Range,
#			self.log_sensor_center
#		)

#		self.proximity_subscriber_center_left = rospy.Subscriber(
#			self.name + '/proximity/center_left',
#			Range,
#			self.log_sensor_center_left
#		)

#		self.proximity_subscriber_left = rospy.Subscriber(
#			self.name + '/proximity/left',
#			Range,
#			self.log_sensor_left
#			)

		# tell ros to call stop when the program is terminated
		rospy.on_shutdown(self.stop)

		# initialize pose to (X=0, Y=0, theta=0)
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
    	"""Subscriber for robot right sensor data"""
    	self.right_sensor = data.range
    	if (data.range < self.threshold):
    		self.object_avoidance = True

	def log_sensor_center_right(self, data):
		"""Subscriber for robot right sensor data"""
		self.center_right_sensor = data.range
		if (data.range < self.threshold):
			self.object_avoidance = True

	def log_sensor_center(self, data):
		"""Subscriber for robot right sensor data"""
		self.center_sensor = data.range
		if (data.range < self.threshold):
			self.object_avoidance = True

	def log_sensor_center_left(self, data):
		"""Subscriber for robot right sensor data"""
		self.center_left_sensor = data.range
		if (data.range < self.threshold):
			self.object_avoidance = True

	def log_sensor_left(self, data):
		"""Subscriber for robot right sensor data"""
		self.left_sensor = data.range
		if (data.range < self.threshold):
			self.object_avoidance = True

    def rotate(self):
		vel_msg = Twist()
		speed = 1
		clockwise = self.rotn_dirn()
		angular_speed = PI*speed/180
		vel_msg.linear.x=0
		vel_msg.linear.y=0
		vel_msg.linear.z=0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0

		# Checking if our movement is CW or CCW
		if clockwise==0:
			vel_msg.angular.z = abs(angular_speed)
		elif clockwise==1:
			vel_msg.angular.z = -abs(angular_speed)
		elif clockwise==2:
			return 0

		prev_value = self.center

		while(self.center <= prev_value):
			velocity_publisher.publish(vel_msg)
			prev_value = self.center

		vel_msg.angular.z = 0
		velocity_publisher.publish(vel_msg)


    def get_control(self):
        return Twist(
            linear=Vector3(
                1,  # moves forward .2 m/s
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
        	# Rotate inplace untill sensor.center keeps decreasing
        	while not self.object_avoidance:
	            # decide control action
	            velocity = self.get_control()

	            # publish velocity message
	            self.velocity_publisher.publish(velocity)

	            # sleep until next step
	            self.rate.sleep()
	        self.rotate()
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