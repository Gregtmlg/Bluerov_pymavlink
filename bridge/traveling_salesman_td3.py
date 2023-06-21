import gym
import numpy as np
import time
from time import sleep
from termcolor import colored
import matplotlib.pyplot as plt

from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

from traveling_salesman_env import UnityEnv

# Set the parameters for the implementation
max_timesteps = 150000  # Maximum number of steps to perform

print(colored("INFO : Gym Environment creation", 'yellow'))
env = UnityEnv('bluerov2_scenario.launch', 256, reward_mode="distance_made")
print(colored("INFO : Gym Environment created", 'yellow'))

sleep(10)
print(colored("INFO : time sleep passed", 'yellow'))
## The noise objects for TD3
# n_actions = env.action_space.shape[-1]
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

start = time.time()
model = TD3(MlpPolicy, env, train_freq=1000, device="cuda")
print(colored("INFO : TD3 model created", 'yellow'))
model.learn(total_timesteps=max_timesteps, log_interval=50, progress_bar=True)
model.save("traveling_salesman")
end = time.time()

print(colored("Temps de process : " + str(end - start), 'green'))

plt.figure()
plt.plot(env.reward_history)
plt.show()

# model = TD3.load('model_bluerov2_td3')
# obs = env.reset()

# while True:
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
    # env.render()