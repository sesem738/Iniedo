#!/usr/bin/env python3

import os
import sys
import time
import math
import random
import numpy as np
import gymnasium as gym
from collections import deque
from squaternion import Quaternion

import rospy
import subprocess
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

sys.path.append('/home/sesem/WorldWideWeb/Iniedo/gazebo/src/iniedo')

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from utils.waypoints_generator import Waypoint_Generator

class GazeboEnv(gym.Env):
    """Gazebo Environment That Follow Gym Interfaces"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(GazeboEnv, self).__init__()
        
        self.imu_data = None
        self.waypoints = None
        self.husky_state = None
        self.wp_threshold = 0.1
        self.way_pt = Waypoint_Generator()
        self.imu_buffer = deque([np.zeros(6) for _ in range(360)], maxlen=360)
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(2,))
        observation_space_dict = {
            'imu_buffer': gym.spaces.Box(low=-math.inf, high=math.inf, shape=(360,6), dtype=np.float32),
            'pose_estimate': gym.spaces.Box(low=-math.inf, high=math.inf, shape=(7,), dtype=np.float32),
            'waypoints': gym.spaces.Box(low=-math.inf, high= math.inf, shape=(1,), dtype=np.float32)
        }
        self.observation_space = gym.spaces.Dict(observation_space_dict)

        rospy.init_node('TestidoEnv', anonymous=True)

        # ROS Services
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # ROS Subscribers
        self.raw_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.husky_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        
        # ROS Publishers
        self.cmd_vel_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException as e:
            print("Pause Physics Failed")
  
        
    def imu_callback(self, imu):
        linear_acceleration = (imu.linear_acceleration.x,
                           imu.linear_acceleration.y,
                           imu.linear_acceleration.z)
        
        angular_velocity = (imu.angular_velocity.x,
                            imu.angular_velocity.y,
                            imu.angular_velocity.z)
        
        self.imu_data = np.asarray([*linear_acceleration, *angular_velocity])
        self.imu_buffer.append(self.imu_data)
        

    def state_callback(self, states):
        id = states.name.index('husky')
        position = (states.pose[id].position.x,
                    states.pose[id].position.y,
                    states.pose[id].position.z)
        
        quat = (states.pose[id].orientation.w,
                states.pose[id].orientation.x,
                states.pose[id].orientation.y,
                states.pose[id].orientation.z
                )

        self.husky_state = np.asarray([*position, *quat])
        

    def reset(self):
        # Resets The Husky and Returns an Initial Observation
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_world()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        # Reset Variables
        self.done = False
        self.imu_data = None
        self.husky_state = None
        self.current_waypt_idx = 0
        random_angle = random.uniform(-math.pi, math.pi) # Test -pi to pi

        # Reset Husky Position
        husky_pose = ModelState()
        husky_pose.model_name = 'husky'
        husky_pose.pose.position.x = round(random.uniform(-10,10), 1) 
        husky_pose.pose.position.y = round(random.uniform(-10,10), 1) 
        husky_pose.pose.orientation.w = math.cos(random_angle / 2)
        husky_pose.pose.orientation.z = math.sin(random_angle / 2)
        self.set_state(husky_pose)

        self.waypoints = self.way_pt.choose_random_trajectory([husky_pose.pose.position.x, husky_pose.pose.position.y])

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print("Unpause Physics Failed")

        time.sleep(0.05)
      
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException as e:
            print("Pause Physics Failed")

        init_pose = self.husky_state
        imu_buffer = np.asarray(list(self.imu_buffer))
        waypoints = self.waypoints[self.current_waypt_idx]

        obs = {
            "imu_buffer": imu_buffer,
            "pose_estimate": init_pose,
            "waypoints": waypoints
        }
        return obs
    
    
    def step(self, pose_estimate, action):

        cmd_vel = Twist()
        cmd_vel.linear.x = action[0]
        cmd_vel.angular.z = action[1]
        self.cmd_vel_pub.publish(cmd_vel)

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print("Unpause Physics Failed")

        time.sleep(0.05)

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException as e:
            print("Pause Physics Failed")

        obs = self._get_obs(pose_estimate)
        rewards = self._compute_rewards(pose_estimate)

        return obs, rewards, self.done, {}


    def _get_obs(self, pose_estimate):
        imu_buffer = np.asarray(list(self.imu_buffer))

        waypoints = self.waypoints[self.current_waypt_idx]
        distance = np.linalg.norm(np.array(pose_estimate[:2]) - np.array(waypoints))
        
        #TODO: Handle end of waypoint
        # Proposed approach: Repeat last waypoint twice
        if distance <= self.wp_threshold:
            if self.current_waypt_idx >= len(self.waypoints): # - repeats 
                self.done = True
            self.current_waypt_idx += 1
            # waypoints = self.waypoints[self.current_waypt_idx : self.current_waypt + 3]
            waypoints = self.waypoints[self.current_waypt_idx]
        
        obs = {
            "imu_buffer": imu_buffer,
            "pose_estimate": pose_estimate,
            "waypoints": waypoints
        }
        return obs

    def _compute_rewards(self,pose_estimate):
        current_waypoint = self.waypoints[self.current_waypt_idx]
        distance = -np.linalg.norm(np.array(pose_estimate[:2]) - np.array(current_waypoint))
        return distance

if __name__ == "__main__":
    env = GazeboEnv()
    _ = env.reset()
    env.step(env.observation_space.sample()['pose_estimate'], env.action_space.sample())
    rospy.spin()
