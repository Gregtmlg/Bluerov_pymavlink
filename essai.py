import numpy as np
import math
from matplotlib import pyplot as plt
import gym
from gym import spaces
from gym.spaces import Box, Dict, Discrete, MultiBinary, MultiDiscrete



import numpy as np
import matplotlib.pyplot as plt
from random import *
from time import *
import matplotlib.pyplot as plt



observation_space = Dict(
            {
                #X Y Z  depart
                'pos_départ' : spaces.Box(low=float("-inf"), high=float("inf"), shape=(1,3), dtype=np.float32),
                #X Y Z arrive
                'intervention' : spaces.Box(low=float("-inf"), high=float("inf"), shape=(7,3), dtype=np.float32),
                #X Y Z ma_pos
                'pos_cur' : spaces.Box(low=float("-inf"), high=float("inf"), shape=(1,3), dtype=np.float32),
                #Niveau de batterie 
                'nivbat' : spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32),
                #Incertitude de position
                'pos_error' : spaces.Box(low=0.0, high=float('inf'), shape=(1,), dtype=np.float32),
                #X Y Z des portes des zones
                'waypnt' : spaces.Box(low=float("-inf"), high=float("inf"), shape=(144, 3), dtype=np.float32),
                #data du velodymne, a determiner 
                'data_v' : spaces.Box(low=float("-inf"), high=float("inf"), shape=(50, 3), dtype=np.float32),
                # data caméra RGB : il faudra mettre le format de l'image
                'data_cam' : spaces.Box(low=0.0, high=255, shape=(10, 5, 3), dtype=np.int16)
            }
            )

action_space =  Box(low=np.array([0,0]), high=np.array([4,144]), dtype=np.uint32)

print(type(action_space.sample()[1]))


