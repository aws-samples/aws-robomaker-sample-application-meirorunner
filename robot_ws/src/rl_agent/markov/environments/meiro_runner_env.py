#
# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

from __future__ import print_function

import time

import boto3
# gym
import gym
import numpy as np
from gym import spaces
from PIL import Image
import os
import random
import math
import sys

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan

TRAINING_IMAGE_SIZE = 360       # Use Lidar scanner which look around 360 degree
LIDAR_SCAN_MAX_DISTANCE = 5.0   # Max distance Lidar scanner can measure
CRASH_DISTANCE = 0.13           # Min distance to obstacle

# REWARD ENUM
CRASHED = 0
MAX_STEPS = 10000
FINISH_LINE = 2.4
FINISHED = 100000.0

# Initial position of the robot
INITIAL_POS_X = 0.0
INITIAL_POS_Y = 0.0

#Size of the stage (M)
STAGE_X_MIN = -5.0   
STAGE_Y_MIN = -5.0
STAGE_X_MAX = 5.0
STAGE_Y_MAX = 5.0

# Footsteps maker is the marker to track the place the robots has passed.
# Define the size of the marker (M)
FOOTSTEPS_MARKER_SIZE = 0.2

# SLEEP INTERVALS
SLEEP_AFTER_RESET_TIME_IN_SECOND = 0.5
SLEEP_BETWEEN_ACTION_AND_REWARD_CALCULATION_TIME_IN_SECOND = 0.3 # LIDAR Scan is 5 FPS (0.2sec).  
SLEEP_WAITING_FOR_IMAGE_TIME_IN_SECOND = 0.01

### Gym Env ###
class TurtleBot3MeiroRunnerEnv(gym.Env):
    def __init__(self):

        self.on_track = 0
        self.progress = 0
        self.yaw = 0
        self.x = INITIAL_POS_X
        self.y = INITIAL_POS_Y
        self.steps = 0
        self.ranges = None
        self.footsteps_marker = np.zeros((int((STAGE_X_MAX - STAGE_X_MIN) / FOOTSTEPS_MARKER_SIZE)+1, int((STAGE_Y_MAX - STAGE_Y_MIN) / FOOTSTEPS_MARKER_SIZE)+1))

        # actions -> steering angle, throttle
        self.action_space = spaces.Box(low=np.array([-1, 0]), high=np.array([+1, +1]), dtype=np.float32)

        # environment -> lider scan
        high = np.array([LIDAR_SCAN_MAX_DISTANCE] * TRAINING_IMAGE_SIZE)
        low = np.array([0.0] * TRAINING_IMAGE_SIZE)
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        
        #ROS initialization
        self.ack_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        self.gazebo_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.init_node('rl_coach', anonymous=True)

        #Subscribe to ROS topics and register callbacks
        rospy.Subscriber('/odom', Odometry, self.callback_pose)
        rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        self.aws_region = rospy.get_param('ROS_AWS_REGION')

        self.reward_in_episode = 0
        self.steps = 0
        self.last_min_distance = sys.maxsize
        self.last_position_x = self.x
        self.last_position_y = self.y
        self.last_footsteps_mark_position = self.calc_footsteps_mark_position(self.x, self.y)


    def reset(self):
        print('Total Reward Reward=%.2f' % self.reward_in_episode,
              'Total Steps=%.2f' % self.steps)
        self.send_reward_to_cloudwatch(self.reward_in_episode)

        self.reward = None
        self.done = False
        self.next_state = None
        self.ranges= None
        self.steps = 0
        self.reward_in_episode = 0

        self.send_action(0, 0) # set the throttle to 0
        self.turtlebot3_reset()

        self.infer_reward_state([0,0])
        return self.next_state

    def turtlebot3_reset(self):
        rospy.wait_for_service('gazebo/set_model_state')

        self.x = INITIAL_POS_X
        self.y = INITIAL_POS_Y
        self.footsteps_marker = np.zeros((int((STAGE_X_MAX - STAGE_X_MIN) / FOOTSTEPS_MARKER_SIZE)+1, int((STAGE_Y_MAX - STAGE_Y_MIN) / FOOTSTEPS_MARKER_SIZE)+1))

        # Put the turtlebot at the initial position
        modelState = ModelState()
        modelState.pose.position.z = 0
        modelState.pose.orientation.x = 0
        modelState.pose.orientation.y = 0
        modelState.pose.orientation.z = 0
        modelState.pose.orientation.w = 0
        modelState.twist.linear.x = 0
        modelState.twist.linear.y = 0
        modelState.twist.linear.z = 0
        modelState.twist.angular.x = 0
        modelState.twist.angular.y = 0
        modelState.twist.angular.z = 0
        modelState.model_name = 'turtlebot3_burger'
        modelState.pose.position.x = self.x
        modelState.pose.position.y = self.y
        self.gazebo_model_state_service(modelState)

        self.last_min_distance = sys.maxsize
        self.last_position_x = self.x
        self.last_position_y = self.y
        self.last_footsteps_mark_position = self.calc_footsteps_mark_position(self.x, self.y)

        time.sleep(SLEEP_AFTER_RESET_TIME_IN_SECOND)

    def step(self, action):
        #initialize rewards, next_state, done
        self.reward = None
        self.done = False
        self.next_state = None

        steering = float(action[0])
        throttle = float(action[1])
        self.steps += 1
        self.send_action(steering, throttle)
        time.sleep(SLEEP_BETWEEN_ACTION_AND_REWARD_CALCULATION_TIME_IN_SECOND)
        self.infer_reward_state(action)

        info = {} #additional data, not to be used for training
        return self.next_state, self.reward, self.done, info

    def callback_scan(self, data):
        self.ranges = data.ranges

    def callback_pose(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

    def send_action(self, steering, throttle):
        speed = Twist()
        speed.linear.x = throttle
        speed.angular.z = steering
        self.ack_publisher.publish(speed)

    def infer_reward_state(self,action):
        #Wait until we have an image from the LIDAR.
        while not self.ranges:
            time.sleep(SLEEP_WAITING_FOR_IMAGE_TIME_IN_SECOND)

        steering = float(action[0])
        throttle = float(action[1])

        #Fit data size to training image
        size = len(self.ranges)
        x = np.linspace(0, size-1, TRAINING_IMAGE_SIZE)
        xp = np.arange(size)
        state = np.clip(np.interp(x, xp, self.ranges), 0, LIDAR_SCAN_MAX_DISTANCE)
        state[np.isnan(state)] = LIDAR_SCAN_MAX_DISTANCE

        #Find min distance 
        min_distance = np.amin(state)

        reward = 0
        if self.last_position_x >= FINISH_LINE:
            print("Congratulations! You passed the finish line!")
            if self.steps == 0:
                reward = 0.0
                done = False
            else:
                reward = FINISHED / self.steps
                done = True
                
        elif min_distance < CRASH_DISTANCE:
            # Robot likely hit the wall
            reward = CRASHED
            done = True
            
        else:
            # When robot is close to the wall, give score based by how far from the wall. 
            # doubled the score when robot is leaving from the wall.
            if min_distance < 0.19:
                
                if min_distance < 0.15:
                    reward = 0.05                
                elif min_distance < 0.17:
                    reward = 0.15                
                else:
                    reward = 0.35
                    
                if min_distance - self.last_min_distance - 0.01 > 0:
                    reward *= 2
                    
                done = False
            else:
                # While the robot is away enough from the wall, give throttle value as score. 
                # (Max throttle 0.1 => 1.0 for score)
                reward = throttle * 10.0 
                done = False

        # leave footstep marker to the place robot has passed through.
        footstep_marker = self.calc_footsteps_mark_position(self.x, self.y)
        if not self.last_footsteps_mark_position == footstep_marker:
            # if the robot had been already walked through that area more than twice, treat it as crashing.
            if self.footsteps_marker[footstep_marker[0]][footstep_marker[1]] > 1:
                reward = CRASHED
                done = True
            # if the robot had been already walked through that area, reduce the reward.
            elif self.footsteps_marker[footstep_marker[0]][footstep_marker[1]] > 0:
                reward = reward * 0.01
                
            self.footsteps_marker[footstep_marker[0]][footstep_marker[1]] += 1
            self.last_footsteps_mark_position = footstep_marker
            
        self.reward_in_episode += reward
        print('Step No=%.2f' % self.steps,
              'Reward=%.2f' % reward,
              'Distance from finish line=%f' % abs(FINISH_LINE - self.x))                    
                
        self.reward = reward
        self.done = done
        self.next_state = state
        
        self.last_min_distance = min_distance
        self.last_position_x = self.x
        self.last_position_y = self.y


    def send_reward_to_cloudwatch(self, reward):
        session = boto3.session.Session()
        cloudwatch_client = session.client('cloudwatch', region_name=self.aws_region)
        cloudwatch_client.put_metric_data(
            MetricData=[
                {
                    'MetricName': 'MeiroRunnerEpisode',
                    'Unit': 'None',
                    'Value': reward
                },
            ],
            Namespace='AWSRoboMakerSimulation'
        )

    def calc_footsteps_mark_position(self, x,y):
        mx = int((x - STAGE_X_MIN)/ FOOTSTEPS_MARKER_SIZE)
        my = int((y - STAGE_Y_MIN)/ FOOTSTEPS_MARKER_SIZE)
        
        if mx < 0 : mx = 0
        elif mx >= len(self.footsteps_marker) : mx = len(self.footsteps_marker) - 1

        if my < 0 : my = 0
        elif my >= len(self.footsteps_marker[0]) : my = len(self.footsteps_marker[0]) - 1
        
        return mx, my
        
class TurtleBot3MeiroRunnerDiscreteEnv(TurtleBot3MeiroRunnerEnv):
    def __init__(self):
        TurtleBot3MeiroRunnerEnv.__init__(self)

        # actions -> straight, left, right
        self.action_space = spaces.Discrete(5)

    def step(self, action):
        # Convert discrete to continuous
        if action == 0:  # turn left
            steering = 1.159
            throttle = 0.08
        elif action == 1:  # turn right
            steering = -1.159
            throttle = 0.08
        elif action == 2:  # straight
            steering = 0
            throttle = 0.1
        elif action == 3:  # steer to the left
            steering = 0.6
            throttle = 0.09
        elif action == 4:  # steer to the right
            steering = -0.6
            throttle = 0.09
        else:  # should not be here.
            raise ValueError("Invalid action")

        continous_action = [steering, throttle]

        return super().step(continous_action)
