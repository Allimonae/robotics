#!/usr/bin/env python3 
import rospy

# Import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist

# Import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry

# Import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion

# Import some useful mathematical operations (and pi):
from math import sqrt, pow, pi

class Square:
    def callback_function(self, topic_data: Odometry):
        # Obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # Obtain the robot's position coordinates:
        pos_x = position.x
        pos_y = position.y

        # Convert orientation coordinates to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z:
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

        # If this is the first time that the callback_function has run,
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation):
        if self.startup:
            # Don't initialize again:
            self.startup = False
            # Set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        node_name = "move_square"

        # A flag if this node has just been launched:
        self.startup = True

        # This might be useful in the main_loop() (to switch between turning and moving forwards):
        self.turn = False

        # Setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(50)  # Hz

        # Define the robot pose variables and initialize them to zero:
        # Variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0

        # Variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        # Define a Twist message instance to set robot velocities:
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"The {node_name} node has been initialized...")

    def shutdownhook(self):
        # Publish an empty Twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True

    def main_loop(self):

        for j in range(2):
            self.x0 = self.x 
            self.y0 = self.y
            self.theta_z0 = self.theta_z
        
            # repeat 4 times to make a square
            for i in range(4):
                # With the robot stationary, read the odometry to determine its current X and Y position in the environment. 
                self.vel.linear.x = 0.15
                self.vel.angular.z = 0.0

                start_x = self.x
                start_y = self.y

                # Move forwards until the robot's X and Y position indicate that it has moved linearly by 0.5m. 
                while not self.ctrl_c:
                    # Here is where your code would go to control the motion of your
                    # robot. Add code here to make your robot move in a square of
                    # dimensions 1 x 1m.
                    dx = abs(self.x - start_x)
                    dy = abs(self.y - start_y)
                    distance_traveled = sqrt(pow(dx,2) + pow(dy, 2))

                    if distance_traveled >= 0.96:
                        # stop moving forwards
                        self.vel.linear.x = 0.0
                        self.pub.publish(self.vel)
                        rospy.sleep(1)
                        break     

                    if distance_traveled >= 0.9:
                        self.vel.linear.x = 0.05           

                    # Publish whatever velocity command has been set in your code above:
                    self.pub.publish(self.vel)
                    # Maintain the loop rate @ 10 Hz:
                    self.rate.sleep()

                rospy.loginfo(f"Moved straight 1 meter.")                

                # Read the robot's odometry to determine its current orientation ("yaw"/θz). 
                self.vel.angular.z = 0.3
                self.theta_z0 = self.theta_z

                angle = (self.theta_z0 + pi/2) % (2 * pi)
                max_angular_speed = 0.3
                kp = 0.5

                # Turn on the spot until the robot's orientation changes by 90°
                while not self.ctrl_c:
                    angle_diff = (angle - self.theta_z + pi) % (2 * pi) - pi

                    if abs(angle_diff) <= 0.04:
                        # stop turning
                        self.vel.angular.z = 0.0
                        self.pub.publish(self.vel)
                        rospy.sleep(1)
                        break

                    self.vel.angular.z = max(0.05, min(max_angular_speed, kp * abs(angle_diff)))
                    self.pub.publish(self.vel)
                    self.rate.sleep()
                    rospy.sleep(0.5)
                
                rospy.loginfo(f"Made a 90 degree turn.")  

            rospy.loginfo("Square complete.")


if __name__ == "__main__":
    node = Square()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass