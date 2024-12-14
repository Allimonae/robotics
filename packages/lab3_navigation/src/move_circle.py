#!/usr/bin/env python3 

# A simple ROS publisher node in Python 
import rospy  
from std_msgs.msg import String  
from geometry_msgs.msg import Twist

class Publisher():  
    def __init__(self):  
        self.node_name = "circle_publisher"  

        #Your Python node needs to publish Twist messages to the /cmd_vel topic in order to make the TurtleBot3 move.
        topic_name = "/cmd_vel"  
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)  
        
        rospy.init_node(self.node_name, anonymous=True)  
        self.rate = rospy.Rate(10)  
        self.ctrl_c = False  
        rospy.on_shutdown(self.shutdownhook)  
        rospy.loginfo(f"The '{self.node_name}' node is active...")  

    # code your shutdownhook() correctly so that the robot stops moving when the node is shutdown (via Ctrl+C in the terminal that launched it)
    def shutdownhook(self):  
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}") 
        self.ctrl_c = True 

        stop_message = Twist()
        stop_message.linear.x = 0.0
        stop_message.linear.y = 0.0
        stop_message.angular.z = 0.0
        self.pub.publish(stop_message)

    def main_loop(self): 
        circular_message = Twist()

        # Make your TurtleBot3 move in a circle with a path radius of approximately 0.5m
        radius = 0.5
        linear_velocity = 0.26
        angular_velocity = linear_velocity/radius

        # robots have a maximum linear velocity (linear.x) of 0.26 m/s, and a maximum angular velocity (angular.z) of 1.82 rad/s. 
        if angular_velocity > 1.82:
            angular_velocity = 1.82
            linear_velocity = angular_velocity * radius

        circular_message.linear.x = linear_velocity
        circular_message.angular.z = angular_velocity

        rospy.loginfo(f"Linear Velocity: {linear_velocity} m/s, Angular Velocity: {angular_velocity} rad/s")

        while not self.ctrl_c:  
            self.pub.publish(circular_message) 
            self.rate.sleep() 

if __name__ == '__main__':  
    publisher_instance = Publisher()  
    try: 
        publisher_instance.main_loop()  
    except rospy.ROSInterruptException: 
        pass 