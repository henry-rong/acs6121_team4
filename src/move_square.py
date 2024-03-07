#!/usr/bin/env python3

import rospy

# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist

# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry

# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion

# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi

class Square():
    def callback_function(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        # convert orientation co-ords to roll, pitch & yaw 
        # (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z
        # so assign these to class variables (so that we can
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        node_name = "move_square"
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() (to switch between 
        # turning and moving forwards)
        self.turn = False

        # setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")


    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True

    def main_loop(self):
        i = 1
        while not self.ctrl_c:
            # here is where your code would go to control the motion of your
            # robot. Add code here to make your robot move in a square of
            # dimensions 1 x 1m...

            if (i == 1):
                if(self.x <= 1):
                    self.vel.linear.x = 0.1
                    self.vel.angular.z = 0
                else:
                    self.vel.linear.x = 0
                    i = 2
            elif (i == 2):
                if(self.theta_z < 1.57):        # Turn 1
                    self.vel.angular.z = 0.2
                else:
                    self.vel.angular.z = 0
                    i = 3
            elif (i == 3):
                if(self.y <= 1):
                    self.vel.linear.x = 0.1
                else:
                    self.vel.linear.x = 0
                    i = 4
            elif (i == 4):
                if (self.theta_z > 0):          # Turn 2
                    self.vel.angular.z = 0.2
                else:
                    self.vel.angular.z = 0
                    i = 5
            elif (i == 5):
                if(self.x >= 0):
                    self.vel.linear.x = 0.1
                else:
                    self.vel.linear.x = 0
                    i = 6
            elif (i == 6):
                if(self.theta_z < -1.57):      # Turn 3
                    self.vel.angular.z = 0.2
                else:
                    self.vel.angular.z = 0
                    i = 7
            elif (i == 7):
                if(self.y >= 0):
                    self.vel.linear.x = 0.1
                else:
                    self.vel.linear.x = 0
                    i = 8
            elif (i == 8):
                if(self.theta_z < 0):
                    self.vel.angular.z = 0.2
                else:
                    self.vel.angular.z = 0
                    i = 1

            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == "__main__":
    node = Square()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass