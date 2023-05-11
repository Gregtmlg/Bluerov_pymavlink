import gym
import numpy as np
from time import sleep
from termcolor import colored

from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

from stable_baselines_env import UnityEnv

# Set the parameters for the implementation
max_timesteps = 1024  # Maximum number of steps to perform

print(colored("INFO : Gym Environment creation", 'yellow'))
env = UnityEnv('bluerov2_scenario.launch', 256)
print(colored("INFO : Gym Environment created", 'yellow'))

sleep(20)
print(colored("INFO : time sleep passed", 'yellow'))
## The noise objects for TD3
# n_actions = env.action_space.shape[-1]
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model = TD3(MlpPolicy, env)
print(colored("INFO : TD3 model created", 'yellow'))
model.learn(total_timesteps=max_timesteps, log_interval=10)
model.save("model_bluerov2_td3")


# obs = env.reset()

# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()