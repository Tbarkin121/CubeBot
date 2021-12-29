#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 18 22:05:15 2021

@author: vegas
"""

import gym
import numpy as np
import math
import pybullet as p
from box_pendulum.resources.cubebot import CubeBot
from box_pendulum.envs.cube_stand_env import CubeStandEnv
from box_pendulum.envs.cube3D_stand_env import Cube3DStandEnv
import matplotlib.pyplot as plt
from time import sleep
werwer
#%%
env = Cube3DStandEnv()

#%%

obs = env.reset()
print('observ = {}'.format(obs))
#%%
for _ in range(50):
    obs, reward, done, info = env.step([0.0, 0.0, 0.0])
    # print('action = {}'.format(action))
    print('observ = {}'.format(obs))
    print('reward = {}'.format(reward))
    sleep(1/240)
#%%
env.close()


#%%
     

# import os 

# import torch
# import pybullet_envs
# import gym
# import numpy as np

# import box_pendulum
# import time

# import torch as th
# from stable_baselines3 import SAC, PPO
# from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, DummyVecEnv
# from stable_baselines3.common.utils import set_random_seed
# from stable_baselines3.common.monitor import Monitor
# from typing import Callable

# env = VecNormalize.load(env_stats_path, env)
# obs = env.reset()
# model = PPO.load('model_save/testsave.zip')
# model.set_env(env)       

