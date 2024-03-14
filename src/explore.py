#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

class Publisher(): 

    def __init__(self): 
        self.node_name = "explore" # name of initialised node
        topic_name = "/move_base_simple/goal" # the message to publish
        self.pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10) 
        
        self.pub2 = rospy.Publisher("cmd_vel", Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        vel_cmd = Twist()
        vel_cmd.linear.x = 0
        vel_cmd.angular.z = 0
        self.pub2.publish(vel_cmd)

        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main_loop(self):

        

        set_position = PoseStamped()

        set_position.header.frame_id = "map"

        set_position.pose.position.x = 0
        set_position.pose.position.y = 1
        set_position.pose.orientation.w = 1



        while not self.ctrl_c: 
            self.pub.publish(set_position)
            self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = Publisher() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass