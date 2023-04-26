import gym
from gym import spaces
from gym.spaces import Box, Dict, Discrete, MultiBinary, MultiDiscrete
import copy
import rospy
import subprocess
from os import path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from numpy import inf
import numpy as np
import random
import math
from gazebo_msgs.msg import ModelState
from squaternion import Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Empty
import os
import time
from random import *
from position_case import type_case
# from Dynamic_Approach.traject3d import evitement
# from Gridy_drone_swipp.Gridy_based import scan

from bridge.bluerov_node import BlueRov

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

COLLISION_DIST = 0.035
GOAL_REACHED_DIST = 0.03



class UnityEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, launchfile, max_episode_length=256):

        #init des different parametre comme positon
        #goal, batterie ect..
        self.bluerov = BlueRov(device='udp:localhost:14551')
        self._max_episode_length = max_episode_length

        self.flag_change_goal=False

        self.flag_change_bat=False
        self.flag_courant=False
        self.flag_change_bat_init=False

        self.step_counter = 0
        self.goal_atteint= np.array([[0],[0],[0],[0],[0],[0],[0]])
        self.position=[0,0,20]
        self.position_depart=np.array([0,0,20])

        self.position_goal=[
                            [0,0,20],
                            [0,0,20],
                            [0,0,20], 
                            [0,0,20],
                            [0,0,20],
                            [0,0,20],
                            [0,0,20]
                           ]
        self.current_dist_to_goal=np.array([
                                    [0],
                                    [0],
                                    [0],
                                    [0],
                                    [0],
                                    [0],
                                    [0]
                                  ])
        self.pre_dist_to_goal=np.array([
                                    [0],
                                    [0],
                                    [0],
                                    [0],
                                    [0],
                                    [0],
                                    [0]
                                  ])
        self.insertitude=0.20
        self.grid =[[-60, -70, 6], [-40, -70, 6], [-20, -70, 6], [0, -70, 6], [20, -70, 6], [40, -70, 6], [60, -70, 6], [80, -70, 6], 
                    [-70, -60, 6], [-50, -60, 6], [-30, -60, 6], [-10, -60, 6], [10, -60, 6], [30, -60, 6], [50, -60, 6], [70, -60, 6], [90, -60, 6], 
                    [-60, -50, 6], [-40, -50, 6], [-20, -50, 6], [0, -50, 6], [20, -50, 6], [40, -50, 6], [60, -50, 6], [80, -50, 6], 
                    [-70, -40, 6], [-50, -40, 6], [-30, -40, 6], [-10, -40, 6], [10, -40, 6], [30, -40, 6], [50, -40, 6], [70, -40, 6], [90, -40, 6], 
                    [-60, -30, 6], [-40, -30, 6], [-20, -30, 6], [0, -30, 6], [20, -30, 6], [40, -30, 6], [60, -30, 6], [80, -30, 6], 
                    [-70, -20, 6], [-50, -20, 6], [-30, -20, 6], [-10, -20, 6], [10, -20, 6], [30, -20, 6], [50, -20, 6], [70, -20, 6], [90, -20, 6], 
                    [-60, -10, 6], [-40, -10, 6], [-20, -10, 6], [0, -10, 6], [20, -10, 6], [40, -10, 6], [60, -10, 6], [80, -10, 6], 
                    [-70, 0, 6], [-50, 0, 6], [-30, 0, 6], [-10, 0, 6], [10, 0, 6], [30, 0, 6], [50, 0, 6], [70, 0, 6], [90, 0, 6], 
                    [-60, 10, 6], [-40, 10, 6], [-20, 10, 6], [0, 10, 6], [20, 10, 6], [40, 10, 6], [60, 10, 6], [80, 10, 6], 
                    [-70, 20, 6], [-50, 20, 6], [-30, 20, 6], [-10, 20, 6], [10, 20, 6], [30, 20, 6], [50, 20, 6], [70, 20, 6], [90, 20, 6], 
                    [-60, 30, 6], [-40, 30, 6], [-20, 30, 6], [0, 30, 6], [20, 30, 6], [40, 30, 6], [60, 30, 6], [80, 30, 6], 
                    [-70, 40, 6], [-50, 40, 6], [-30, 40, 6], [-10, 40, 6], [10, 40, 6], [30, 40, 6], [50, 40, 6], [70, 40, 6], [90, 40, 6], 
                    [-60, 50, 6], [-40, 50, 6], [-20, 50, 6], [0, 50, 6], [20, 50, 6], [40, 50, 6], [60, 50, 6], [80, 50, 6], 
                    [-70, 60, 6], [-50, 60, 6], [-30, 60, 6], [-10, 60, 6], [10, 60, 6], [30, 60, 6], [50, 60, 6], [70, 60, 6], [90, 60, 6], 
                    [-60, 70, 6], [-40, 70, 6], [-20, 70, 6], [0, 70, 6], [20, 70, 6], [40, 70, 6], [60, 70, 6], [80, 70, 6], 
                    [-70, 80, 6], [-50, 80, 6], [-30, 80, 6], [-10, 80, 6], [10, 80, 6], [30, 80, 6], [50, 80, 6], [70, 80, 6], [90, 80, 6], 
                    [-60, 90, 6], [-40, 90, 6], [-20, 90, 6], [0, 90, 6], [20, 90, 6], [40, 90, 6], [60, 90, 6], [80, 90, 6]]

        #fonction qui place le goal au hasard parmis les waypoints de la grille
        self.change_goal()
        self.batterie=self.bluerov.get_battery_percentage()
        self.batterie_pre=self.batterie
        # definition de l'espace d'action et d'observation
        gym.Env.__init__(self)


        # On a déterminé 4 actions possibles pour le robot : 3 qui gèrent le mode de déplacement (scan, evit, recalibrage),
        # 1 qui gère la zone à rejoindre avec le mode de déplacement choisis.
        self.action_space =  Dict(
            {
               'mode_trait' : spaces.Discrete(3),
               'case_suivante' : spaces.Discrete(len(self.grid))
                
            })

        self.observation_space = Dict(
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

        port = '11311'
        subprocess.Popen(["roscore", "-p", port])

        print("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node('TD3')
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        print("Gazebo launched!")
        

    #Actualisaiton de l'observation 
    def get_observation(self):
        
        dict_obs = self.observation_space.sample()
        
        
        #tirage de l'aléa de baisse subite de la batterie 
        self.batterie=self.bluerov.get_battery_percentage()

        #tirage de l'aléa de courant
        if self.flag_courant==True:
            self.pos_error=self.pos_error+0.2
            facteur_courant=2
        else  :
            self.pos_error=self.pos_error+0.1
            facteur_courant=1
        

        #tirage de l'aléa batterie
        if self.flag_change_bat==True and self.flag_change_bat_init==False  : 
            self.batterie_modif=self.batterie/2
            self.flag_change_bat_init==True 
        if self.flag_change_bat==True and  self.flag_change_bat_init==True : 

            self.batterie_modif =  self.batterie_modif - (self.batterie_pre- self.batterie )* facteur_courant
        
        

        #actualisation des position 
        dict_obs['pos_départ'] = np.array([self.position_depart], dtype=np.float32)
        dict_obs['intervention'] = np.array([self.position_goal], dtype=np.float32)
        dict_obs['pos_cur'] = np.array([self.bluerov.current_pose], dtype=np.float32)
        
        #baisse de la batterie simple
        dict_obs['nivbat'] = np.array([self.batterie_modif], dtype=np.float32) 
        dict_obs['pos_error'] = np.array([self.pos_error], dtype=np.float32)
        dict_obs['waypnt'] = np.array(self.grid)
        dict_obs['data_v'] = np.array([self.bluerov.get_collider_obstacles()], dtype=np.float32)
        
        self.batterie_pre=self.batterie
        return Dict(dict_obs)
    
    def reward(self) : 

        score_goal=0
        score_distance=0
        score_batterie=0
        score_dep=0
        score_traitement=0
        score_recalibrage=0
       
        positon_cur= self.bluerov.get_current_pose()
        positon_goal=np.array(self.position_goal)

        self.pre_dist_to_goal =  self.current_dist_to_goal

        #Calcule de la distance du robot par rapport a chaqun des goals 
        current_dist_to_goal=[[],[],[],[],[],[],[]]
        for i in range(0,7) :
             current_dist_to_goal[i] = np.linalg.norm(positon_goal[i]- positon_cur)


        
        for i in range(0,7) :
            #On regade si le robot a atteint un goal si oui +100pts et on pase la variable de goal atteint a 1 pour ce goal 
            if current_dist_to_goal[i]< GOAL_REACHED_DIST : 
                score_goal=100
                self.goal_atteint[i]=1

            #SI le robot a déja atteint les coordonée de ce goal si alors on calcule rien dautre sinon : 
            if self.goal_atteint[i]==0 :
                #on calcule du rapprochement/eloignement effectuer par le robot entre le step n et n+1 : 
                if abs(current_dist_to_start[i] - self.pre_dist_to_goal[i])>0 :

                    score_distance=score_distance+abs(current_dist_to_start[i] - self.pre_dist_to_goal[i])*10
                #de cette maniere on a pas de maluse quand on s'eloigne d'un goal qu'on vient de traiter.

        #calcule de la ditance de par rapport au départ 
        current_dist_to_start = np.linalg.norm(self.position_depart - positon_cur)

        #actualisation de la variable pour le prochain tour de boucle.
        self.current_dist_to_goal=current_dist_to_goal

        
        #si batterie  a 0 et le robot pas a la zone de départ alors note de batterie tres mauvaise                
        if self.batterie==0 :
            #si le robot et a moins de 25 mettre de son départ quand 0 alors bien : 
            if current_dist_to_start < 25 :
                score_batterie=1000 
            #sinon mauvais :
            else :        
                score_batterie = current_dist_to_start*(-100)

        #verif de la case precendante et de laction précédente 
        pso_robot=[self.pos_pre[0],self.pos_pre[1]]
        nbr_case=5

        #Récupere les données lier a la case dans laquel le robot ce trouve, les diffrentes portes possible et laction a effectuer dans le casse
        type, sortie_disp=type_case(nbr_case,pso_robot)

        self.pre_action_tr = action_tr 
        self.pre_action_dep = action_dep
        #verification pour voir si la porte choisi par le robot corrrespond a une des portes disponible dans la casse ou il ce trouvait. 
        for i in range(0,4) : 
            #si oui, bon pts 
            if self.pre_action_dep == sortie_disp[i] :
                score_dep = 100
        #sinon malusse
        if score_dep == 0 : 
            score_dep = -100

        #Verification du mode de traitement choisi par rapport a la casse ou il etait 
        if type==self.pre_action_tr :
            score_traitement=100
        else :
            score_traitement=-100

        #actualisation des variables 
        action_tr= self.cur_action_tr
        action_dep= self.cur_action_dep  

        if  self.pos_error > 1.5 :
            score_recalibrage = -100

        elif self.pos_error > 1 : 
           score_recalibrage=(self.pos_error-1) * -100


        # calcule du score 
        self.score = score_distance+score_goal+score_traitement+score_dep+score_batterie+score_recalibrage
        return self.score




    def step(self, action):

        #nombre de step+1
        self.step_counter += 1
        #on recupère les infos contenue dans le dic "action", deux données, mode de traitement et coordonées voulu apres avoir fini le mode de traitement 
        sample_action=action
        action_trait=sample_action['mode_trait']
        action_dep=sample_action['case_suivante']

        self.cur_action_tr = action_trait
        self.cur_action_dep = self.grid[action_dep-1]

        #position avant de bouger 
        self.pos_pre=self.bluerov.current_pos

        

        #mise en route du mode de traitement choisi
        if action_trait == 1 :
            self.bluerov.do_evit(self.pos_pre, action_dep)

        elif action_trait == 2 :
            self.bluerov.do_scan(self.pos_pre, action_dep)

        elif  action_trait == 4 :
            self.bluerov.do_evit(self.pos_pre, self.position_depart)
            done = True
        # elif  action_trait == 3 :
        #     self.bluerov.do_recalibrage(self.pos_pre, self.pos_pre)
        #     self.pos_error = 0

        self.cur_pos=self.bluerov.current_pos

        self.imponderables()               
        #actualise les infos
        observations = self.get_observation()
        
        #pas capté ce que c'est, mais c'est important 
        info = {}

        #flag de fin d'épisode 
        done = False

        #on teste si on a dépassé le nombre de step max d'un épisode
        if self.step_counter >= self._max_episode_length:
            #si oui fin d'épisode
            done = True

        
        reward=self.reward()
       
        #verifie si une collision a eu lieu 
        collision, min_laser = self.observe_collision(self.velodyne_data)

        #reward negarif si collision + fin d'épisode 
        if collision:
            done = True

       

        #calculer la distance entre le goal et la position actul 
        if self.current_dist_to_goal < GOAL_REACHED_DIST or self.cur_pos :
            done = True
        
        #imprime le reward dans ros et le nombre de step
        rospy.loginfo(reward)
        rospy.loginfo(self.step_counter)
        
        #envoi les infos a l'IA
        return observations, reward, done, info



    # Reset the state of the environment to an initial state
    def reset(self):

        
        self.step_counter = 0
        self.position=[0,0,20]
        self.position_depart=[0,0,20]
        self.position_goal=[
                            [0,0,20],
                            [0,0,20],
                            [0,0,20], 
                            [0,0,20],
                            [0,0,20],
                            [0,0,20],
                            [0,0,20]
                           ]
        self.batterie=10000
        self.flag_change=False
        self.position_pub.publish(self.position)

        self.change_goal()

        observations = self.get_observation()


        return observations

    # Place a new goal and check if its lov\cation is not on one of the obstacles
    def change_goal(self):
        n = randint(1,10)
        #self.position_goal=waypoint_case(n)

    def imponderables(self): 

        self.batterie=self.bluerov.get_battery_percentage()

        #tirage de l'aléa de baisse subite de la batterie 
        if randint(1,1000)==1 and self.flag_change_bat==False:
            self.flag_change_bat=True

        else :
            self.batterie_modif=self.batterie

        #tirage de l'aléa de courant
        if randint(1,500)==1 and self.flag_courant==False:
            self.flag_courant=True   

        





