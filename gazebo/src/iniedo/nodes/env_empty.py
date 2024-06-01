#!/usr/bin/env python3

import os
import sys
import time
import math
import random
import numpy as np
import gymnasium as gym
from squaternion import Quaternion

import rospy
import subprocess
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

sys.path.append('/home/sesem/WorldWideWeb/Iniedo/gazebo/src/iniedo')

from utils.waypoints_generator import Waypoint_Generator
from gazebo_msgs.msg import ModelState, ModelStates

class GazeboEnv(gym.Env):
    """Gazebo Environment That Follow Gym Interfaces"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(GazeboEnv, self).__init__()
        
        self.imu_data = None
        self.waypoints = None
        self.husky_state = None
        self.way_pt = Waypoint_Generator()

        rospy.init_node('TestidoEnv', anonymous=True)

        # ROS Services
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        # ROS Subscribers
        self.raw_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.husky_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        
        # ROS Publishers
        self.set_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.cmd_vel = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)

        # rospy.wait_for_service('/gazebo/pause_physics')
        # try:
        #     self.pause()
        # except rospy.ServiceException as e:
        #     print("Pause Physics Failed")

        
        
    def imu_callback(self, imu):
        linear_acceleration = (imu.linear_acceleration.x,
                           imu.linear_acceleration.y,
                           imu.linear_acceleration.z)
        
        angular_velocity = (imu.angular_velocity.x,
                            imu.angular_velocity.y,
                            imu.angular_velocity.z)
        
        self.imu_data = (linear_acceleration, angular_velocity)
        

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

        quat = Quaternion(*quat)

        euler = quat.to_euler(degrees=True)
        self.husky_state = [position, quat]
        self.yaw = euler[2]
        

    def reset(self):
        # Resets The Husky and Returns an Initial Observation
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_world()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        # Reset Variables
        self.imu_data = None
        self.husky_state = None
        random_angle = random.uniform(-3.14159, 3.14159) # Test -pi to pi
        
        print("Random Angle: ", math.degrees(random_angle))

        # Reset Husky Position
        husky_pose = ModelState()
        husky_pose.model_name = 'husky'
        husky_pose.pose.position.x = round(random.uniform(-1,1)*10, 1) 
        husky_pose.pose.position.y = round(random.uniform(-1,1)*10, 1) 
        husky_pose.pose.position.z = 0.0
        husky_pose.pose.orientation.w = math.cos(random_angle / 2)
        husky_pose.pose.orientation.z = math.sin(random_angle / 2)
        husky_pose.reference_frame = 'world'
        self.set_state.publish(husky_pose)

        self.waypoints = self.way_pt.choose_random_trajectory([husky_pose.pose.position.x, husky_pose.pose.position.y])

        # rospy.wait_for_service('/gazebo/unpause_physics')
        # try:
        #     self.unpause()
        #     print("We are Here")
        # except rospy.ServiceException as e:
        #     print("Unpause Physics Failed")

        # time.sleep(0.5)

        # print("Gazebo Yaw: ", self.yaw)

        # while self.imu_data is None or self.husky_state is None:
        #     print(self.husky_state)
        #     print("Waiting For Message")
        

        # rospy.wait_for_service('/gazebo/pause_physics')
        # try:
        #     self.pause()
        #     print("We are Gone")
        # except rospy.ServiceException as e:
        #     print("Pause Physics Failed")

        # rospy.wait_for_service('/gazebo/unpause_physics')
        # try:
        #     self.unpause()
        #     print("We are Here")
        # except rospy.ServiceException as e:
        #     print("Unpause Physics Failed")

        

        return self.imu_data
    
    
    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print("Unpause Physics Failed")

        while self.imu_data is None or self.husky_state is None:
            print("Waiting For Message")
        
        # TODO: Get_Observations

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException as e:
            print("Pause Physics Failed")


    def render(self, mode='human'):
        pass

    def close(self):
        pass

if __name__ == "__main__":
    env = GazeboEnv()
    env.reset()
    rospy.spin()
