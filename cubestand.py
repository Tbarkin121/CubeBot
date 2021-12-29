import os 

import torch
import pybullet_envs
import gym
import numpy as np

import box_pendulum
import time

import torch as th
from stable_baselines3 import SAC, PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, DummyVecEnv
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.monitor import Monitor
from typing import Callable

def main():
    tb_name = 'TargetAngleVariance03'
    # tb_name = 'AddHingeVelObs'
    env_stats_path = 'model_save/stats_' + tb_name + '.pkl'

    start_fresh = False
    do_training = False
    do_render = True
    num_cpu = 16
    
    
    # nn = torch.nn.Sequential(torch.nn.Linear(8, 64), torch.nn.Tanh(),
    #                          torch.nn.Linear(64, 2))
    # agent = TRPOAgent(policy=nn)
    # agent.load_model("agent.pth")
    # agent.train("SimpleDriving-v0", seed=0, batch_size=5000, iterations=100,
    #             max_episode_length=250, verbose=True)
    # agent.save_model("agent.pth")

    policy_kwargs = dict(activation_fn=th.nn.LeakyReLU, net_arch=[dict(pi=[64, 64, 64], vf=[64, 64, 64])])

    env = SubprocVecEnv([make_env('CubeStand-v0', i, log_dir='log/') for i in range(num_cpu)])
    
    if(start_fresh):
        # env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)
        obs = env.reset()
        model = PPO('MlpPolicy', 
                    env, 
                    policy_kwargs = policy_kwargs, 
                    verbose=1, 
                    tensorboard_log='log/')
    else:
        # env = VecNormalize.load(env_stats_path, env)
        obs = env.reset()
        model = PPO.load('model_save/' + tb_name)
        model.set_env(env)       
    
    if(do_training):
        model.learn(total_timesteps=5000000,
                    reset_num_timesteps=False, #Getting some issues with this line... not sure why
                    tb_log_name=tb_name)
            # callback=callbacks)
        model.save('model_save/' + tb_name)
        # env.save(env_stats_path)
    
    if(do_render):
        eval_env = DummyVecEnv([make_env('CubeStand-v0', i, seed=2, log_dir='log/') for i in range(1)])
        # eval_env = VecNormalize.load(env_stats_path, eval_env)
        # eval_env = gym.make('SimpleDriving-v0')
        obs = eval_env.reset()
        obs=obs[0]
        for _ in range(2000):
            action = model.predict(obs, deterministic=False)[0]
            obs, reward, done, _ = eval_env.step(action)
            obs = obs[0]
            eval_env.render()
            print('action = {}'.format(action))
            print('observ = {}'.format(obs))
            print('reward = {}'.format(reward))
            if done:
                ob = eval_env.reset()
                time.sleep(1/30)


def make_env(env_id: str, rank: int, seed: int = 1, log_dir=None) -> Callable:
    '''
    Utility function for multiprocessed env.
    
    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environment you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    :return: (Callable)
    '''
    def _init() -> gym.Env:
        env = gym.make(env_id)
        
        # Create folder if needed
        if log_dir is not None:
            os.makedirs(log_dir, exist_ok=True)
        
        env = Monitor(env, log_dir)
        env.seed(seed + rank)
        return env
    set_random_seed(seed)
    return _init

if __name__ == '__main__':
    main()
