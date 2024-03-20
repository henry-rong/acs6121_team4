#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Explorer(): 

    def callback(self, data: Odometry):
        self.pose = data.pose.pose
        return self.pose.position


    def __init__(self): 
        self.node_name = "explore" # name of initialised node
        topic_name = "/move_base_simple/goal" # the message to publish
        self.pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10) 
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)       
        self.pub2 = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.pub3 = rospy.Publisher("explore_debug", String, queue_size=10)
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

        t = 0.1 # coordinate tolerance

        def set_position(message,x,y):
            message.header.frame_id = "map"
            message.pose.position.x = x
            message.pose.position.y = y
            message.pose.orientation.w = 1
            return message

        # def convert2ASCII(data_string):
        #     ascii_text = ''.join(chr(int(data, 2)) for data in str(data_string).split())
        #     return ascii_text

        def check_position(position,x,y):
            return (x - t <= position.x <= x + t) and (y - t <= position.y <= y + t)

        locations = [[1.25,1.25],[-1.25,1.25],[-1.25,-1.25],[1.25,-1.25]]

        visit_no = 0

        coordinate = PoseStamped()

        currentPosition = Odometry().pose.pose.position

        while not self.ctrl_c: 

            if visit_no == 4:
                visit_no = 0

            self.pub.publish(set_position(coordinate,locations[visit_no][0],locations[visit_no][1]))

            if check_position(self.pose.position,locations[visit_no][0],locations[visit_no][1]):
                visit_no += 1

            # if location == 0 and check_position(self.pose.position,0,0):
            #     self.pub.publish(set_position(coordinate,1.25,1.25))
            #     # self.pub3.publish(str(check_position(self.pose.position,1.25,1.25)))
            # elif check_position(self.pose.position,1.25,1.25):
            #     location += 1
            #     self.pub.publish(set_position(coordinate,-1.25,1.25))
            #     if check_position(self.pose.position,-1.25,1.25):
            #         self.pub.publish(set_position(coordinate,-1.25,-1.25))
            #         if check_position(self.pose.position,-1.25,-1.25):
            #             self.pub.publish(set_position(coordinate,1.25,-1.25))

if __name__ == '__main__': 
    publisher_instance = Explorer() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass





