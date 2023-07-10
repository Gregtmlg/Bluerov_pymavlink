import gym
import numpy as np
import time
from time import sleep
from termcolor import colored
import matplotlib.pyplot as plt

from stable_baselines3 import TD3, PPO
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

from traveling_salesman_env import TravelingSalesmanEnv

# Set the parameters for the implementation
max_timesteps = 150000  # Maximum number of steps to perform

print(colored("INFO : Gym Environment creation", 'yellow'))
env = TravelingSalesmanEnv(max_steps=500, type_of_use="ardusub", nb_goals=5, action_k=0.1, step_k=1)
print(colored("INFO : Gym Environment created", 'yellow'))

print(colored("INFO : time sleep passed", 'yellow'))
## The noise objects for TD3
# n_actions = env.action_space.shape[-1]
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

start = time.time()
# model = PPO("MlpPolicy", env, device="cuda")

model = PPO.load('travel_sales_10000000_ak_01_sk_1')
# obs = env.reset_2()
# print(env.init_rov_position)
# print(env.goals)
# done = False
# time.sleep(50)
for i in range(10):
    obs = env.reset_2()
    print(env.init_rov_position)
    print(env.goals)
    done = False
    while not done:
        action, _states = model.predict(obs)
        print(action)
        obs, rewards, done, info = env.step(action)
        print(rewards)

env.close()