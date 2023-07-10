import time
from time import sleep
from termcolor import colored
import matplotlib.pyplot as plt

from traveling_salesman_env import TravelingSalesmanEnv

from stable_baselines3 import TD3
from stable_baselines3.td3.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

# Set the parameters for the implementation
max_timesteps = 50000  # Maximum number of steps to perform

print(colored("INFO : Gym Environment creation", 'yellow'))
env = TravelingSalesmanEnv(max_steps=256, type_of_use="ardusub", nb_goals=5)
print(colored("INFO : Gym Environment created", 'yellow'))
start = time.time()
model = TD3(MlpPolicy, env, train_freq=1000)
print(colored("INFO : TD3 model created", 'yellow'))
model.learn(total_timesteps=max_timesteps, log_interval=1, progress_bar=True)
model.save("traveling_salesman")
end = time.time()

print(colored("Temps de process : " + str(end - start), 'green'))

plt.figure()
plt.plot(env.reward_history)
plt.show()

