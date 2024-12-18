#!/usr/bin/env python3

import gym
from gym import spaces
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class TurtleBotEnv(gym.Env):
    def __init__(self):
        super(TurtleBotEnv, self).__init__()

        # Define action and observation space
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=10, shape=(360,), dtype=np.float32)
        
        self.prev_position = np.array([0.0, 0.0])
        self.robot_x = 0.0
        self.robot_y = 0.0
	
        # Initialize lidar data to avoid AttributeError
        self.lidar_data = np.array([10.0] * 360)  # Initial value representing max range

        # ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        rospy.Subscriber('/odom', Odometry, self._odom_callback)

        # Start a ROS node for this environment
        # rospy.init_node('turtlebot_env', anonymous=True)

        # Maximum steps per episode
        # self.max_steps = 1000  # Adjust based on task complexity
        # self.current_step = 0

        # Allow time for initialization and first sensor data update
        rospy.sleep(1)

    def reset_robot_position(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            # Define the robot's starting pose
            start_pose = ModelState()
            start_pose.model_name = 'turtlebot3'  # Model name as in Gazebo
            start_pose.pose.position.x = -3.0
            start_pose.pose.position.y = 1.0
            start_pose.pose.position.z = 0.0
            start_pose.pose.orientation.x = 0.0
            start_pose.pose.orientation.y = 0.0
            start_pose.pose.orientation.z = 0.0
            start_pose.pose.orientation.w = 1.0
            
            self.current_step = 0  # Reset step counter

            set_state(start_pose)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def reset(self):
        # Reset the environment and return an initial observation
        self._stop_robot()  # Stop any movement
        self.reset_robot_position()
        rospy.sleep(1)  # Allow time for sensors to update
        return self._get_state()

    def step(self, action):
        self.current_step += 1  # Increment step counter

        # Execute action
        self._take_action(action)
        rospy.sleep(0.1)  # Small delay to allow action to take effect

        # Get the current state (LIDAR data)
        state = self._get_state()

        # Compute reward and check if the episode is done
        reward, done = self._compute_reward(state, action)

        # End episode if maximum steps reached
        # if self.current_step >= self.max_steps:
        #     done = True

        return state, reward, done, {}

    def _take_action(self, action):   
        # Map continuous action to robot movement
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]  # Linear velocity (forward/backward)
        vel_cmd.angular.z = action[1]  # Angular velocity (turning)
        self.cmd_vel_pub.publish(vel_cmd)

    def _get_state(self):
        # Return the current lidar observation
        return np.clip(self.lidar_data, 0, 10)

    def _compute_reward(self, state, action):
        done = False
        reward = 0

        # Check for collision using LIDAR data
        if np.min(state) < 0.2:  # Too close to an obstacle
            reward = -500 # Large penalty for collision
            done = True  # End the episode
        elif np.min(state) < 0.5:  # Close to an obstacle
            reward -= np.exp((0.5 - np.min(state)) * 10)   # Smaller penalty for being too close
        else:
            # reward for staying alive
            reward += 5.0

            # Compute movement reward
            current_position = np.array([self.robot_x, self.robot_y])
            movement_distance = np.linalg.norm(current_position - self.prev_position)

            if action[0] > 0:  # Forward movement
                reward += movement_distance * 10.0 * np.min(state)
            elif action[0] < 0:  # Backward movement
                reward -= movement_distance * 5.0

            # Penalty for turning
            reward -= abs(action[1]) * 0.5

            # Small penalty for staying in place
            if movement_distance < 0.1:
                reward -=  2.0

            # Exploration reward based on distance from start
            start_position = np.array([0.0, 0.0])
            distance_from_start = np.linalg.norm(current_position - start_position)
            reward += distance_from_start * 3.0

            # Encourage the robot to avoid getting stuck by giving a small penalty if it keeps turning without moving
            # if movement_distance < 0.1 and abs(action[1]) > 0:
            #     reward -= 1.0  # Penalty for turning without moving forward

        # Update the robot's previous position
        self.prev_position = np.array([self.robot_x, self.robot_y])

        return reward, done

    def _scan_callback(self, data):
        # Replace any NaN or inf values in lidar data with a max distance value, e.g., 10.0 meters
        self.lidar_data = np.array([10.0 if np.isnan(x) or np.isinf(x) else x for x in data.ranges])

    def _odom_callback(self, data):
        # Extract the robot's current position from the Odometry message
        self.robot_x = data.pose.pose.position.x
        self.robot_y = data.pose.pose.position.y

    def _stop_robot(self):
        # Send zero velocity to stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

if __name__ == "__main__":
    env = TurtleBotEnv()
    env.reset()
    # for _ in range(100):
        # action = env.action_space.sample()
        # state, reward, done, _ = env.step(action)
        # print(state, reward, done)
        # if done:
        #     env.reset()
