#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt


class ThymioController:

    def __init__(self):
        """Initialization."""

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

        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

        # initialize linear and angular velocities to 0
        self.velocity = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)
        self.init_pose = self.human_readable_pose2d(self.pose)
        self.vTol=.1
        self.vDir=0

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

    def get_control(self,vDir=0):
        if  self.vDir==0:
            vAng = .4
        else:
            vAng= -.4
        return Twist(
            linear=Vector3(
                .5,  # moves forward .2 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                vAng
            )
        )
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        vcurr = self.human_readable_pose2d(self.pose)
        return sqrt(pow((goal_pose[0] - vcurr[0]), 2) +
                    pow((goal_pose[1] - vcurr[1]), 2))

    def run(self):
        """Controls the Thymio."""
        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():

            # decide control action
            velocity = self.get_control()

            # publish velocity message
            self.velocity_publisher.publish(velocity)
            vDis = self.euclidean_distance(self.init_pose)
            if vDis<=self.vTol and rospy.Time.now().to_sec()-t0>7:
                t0=rospy.Time.now().to_sec()
                self.vDir=1-self.vDir    
            self.rate.sleep()

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
