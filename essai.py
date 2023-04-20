import numpy as np
import math

import gym
from gym import spaces
from gym.spaces import Box, Dict, Discrete, MultiBinary, MultiDiscrete

depart = -78
arrivee = 78

linspace = np.linspace(depart, arrivee, 40)
array = np.array([[1,2,3,4,5], [2,3,4,5,6]])
liste = [1,2,3,4,5]
zeros = np.zeros((3,2))

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
                        'waypnt' : spaces.Box(low=float("-inf"), high=float("inf"), shape=(50, 3), dtype=np.float32),
                        #data du velodymne, a determiner 
                        'data_v' : spaces.Box(low=float("-inf"), high=float("inf"), shape=(50, 3), dtype=np.float32),
                    }
                  )

print(observation_space.sample())




 #commentaire 
# tu saoules avec ton commentaire
 import navpy

def gps2ned(point_gps_x, point_gps_y) :

    #prend le premier X Y pour points de référence ned 0,0 pour calculer les autres point 
    lat_ref=point_gps_x[0]
    lon_ref=point_gps_y[0]
    alt_ref=0


    #¢reation des listes qui vont recevoire les coordonnées
    point_ned_x=[]
    point_ned_y=[]

    #calculer de transformation de GPS a NED puis ajouter dans les listes 
    for i in range(1,len(point_gps_x)) : 

        x,y,z=navpy.lla2ned(point_gps_x[i],point_gps_y[i],0 ,lat_ref , lon_ref,alt_ref, latlon_unit='deg', alt_unit='m', model='wgs84')
        point_ned_x.append(x)
        point_ned_y.append(y)
        

    #retour des points NED
    return point_ned_x,point_ned_y

