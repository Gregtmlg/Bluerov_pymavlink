import gym
from gym import spaces
from gym.spaces import Box, Dict, Discrete, MultiBinary, MultiDiscrete
import copy
import rospy
import subprocess
import threading
from os import path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from numpy import inf
import numpy as np
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
import random
from position_case import box_type_exit, coordonnes_cases
# from Dynamic_Approach.traject3d import evitement
# from Gridy_drone_swipp.Gridy_based import scan

from bluerov_node import BlueRov
from termcolor import colored
import torch

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

COLLISION_DIST = 0.035
GOAL_REACHED_DIST = 0.03



class UnityEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self, launchfile, max_episode_length=256):

        #init des different parametre comme positon
        #goal, batterie ect..
        print(colored("INFO : Init Gym Env", 'yellow'))
        # launch ArduSub sitl with threading
        self.ardu_sitl=threading.Thread(name='ArduSub Sitl', target=self.launch_sitl)
        self.ardu_sitl.deamon=True
        self.ardu_sitl.start()

        self.bluerov = BlueRov(device='udp:localhost:14550')
        time.sleep(5)
        self.bluerov.update()
        self._max_episode_length = max_episode_length

        self.flag_change_goal=False
        self.new_goal_areas = []
        self.nb_goals_reached = 0

        self.flag_change_bat=False
        self.flag_courant=False
        self.flag_change_bat_init=False
        self.facteur_courant = 1

        self.step_counter = 0
        self.global_step_counter = 0
        self.goal_reached= False
        self.position=self.bluerov.get_current_pose()
        self.position_depart=self.change_init()
        self.pos_error = 0
        self.last_pos_error = 0
        self.scanning_depth = [-7]
        self.last_waypnt_chosen = self.position_depart
        self.check_waypnt_chose = True

        self.position_goal=self.change_goal()
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
        
        self.grid =[[-60, -70, -7], [-40, -70, -7], [-20, -70, -7], [0, -70, -7], [20, -70, -7], [40, -70, -7], [60, -70, -7], [80, -70, -7], 
                    [-70, -60, -7], [-50, -60, -7], [-30, -60, -7], [-10, -60, -7], [10, -60, -7], [30, -60, -7], [50, -60, -7], [70, -60, -7], [90, -60, -7], 
                    [-60, -50, -7], [-40, -50, -7], [-20, -50, -7], [0, -50, -7], [20, -50, -7], [40, -50, -7], [60, -50, -7], [80, -50, -7], 
                    [-70, -40, -7], [-50, -40, -7], [-30, -40, -7], [-10, -40, -7], [10, -40, -7], [30, -40, -7], [50, -40, -7], [70, -40, -7], [90, -40, -7], 
                    [-60, -30, -7], [-40, -30, -7], [-20, -30, -7], [0, -30, -7], [20, -30, -7], [40, -30, -7], [60, -30, -7], [80, -30, -7], 
                    [-70, -20, -7], [-50, -20, -7], [-30, -20, -7], [-10, -20, -7], [10, -20, -7], [30, -20, -7], [50, -20, -7], [70, -20, -7], [90, -20, -7], 
                    [-60, -10, -7], [-40, -10, -7], [-20, -10, -7], [0, -10, -7], [20, -10, -7], [40, -10, -7], [60, -10, -7], [80, -10, -7], 
                    [-70, 0, -7], [-50, 0, -7], [-30, 0, -7], [-10, 0, -7], [10, 0, -7], [30, 0, -7], [50, 0, -7], [70, 0, -7], [90, 0, -7], 
                    [-60, 10, -7], [-40, 10, -7], [-20, 10, -7], [0, 10, -7], [20, 10, -7], [40, 10, -7], [60, 10, -7], [80, 10, -7], 
                    [-70, 20, -7], [-50, 20, -7], [-30, 20, -7], [-10, 20, -7], [10, 20, -7], [30, 20, -7], [50, 20, -7], [70, 20, -7], [90, 20, -7], 
                    [-60, 30, -7], [-40, 30, -7], [-20, 30, -7], [0, 30, -7], [20, 30, -7], [40, 30, -7], [60, 30, -7], [80, 30, -7], 
                    [-70, 40, -7], [-50, 40, -7], [-30, 40, -7], [-10, 40, -7], [10, 40, -7], [30, 40, -7], [50, 40, -7], [70, 40, -7], [90, 40, -7], 
                    [-60, 50, -7], [-40, 50, -7], [-20, 50, -7], [0, 50, -7], [20, 50, -7], [40, 50, -7], [60, 50, -7], [80, 50, -7], 
                    [-70, 60, -7], [-50, 60, -7], [-30, 60, -7], [-10, 60, -7], [10, 60, -7], [30, 60, -7], [50, 60, -7], [70, 60, -7], [90, 60, -7], 
                    [-60, 70, -7], [-40, 70, -7], [-20, 70, -7], [0, 70, -7], [20, 70, -7], [40, 70, -7], [60, 70, -7], [80, 70, -7], 
                    [-70, 80, -7], [-50, 80, -7], [-30, 80, -7], [-10, 80, -7], [10, 80, -7], [30, 80, -7], [50, 80, -7], [70, 80, -7], [90, 80, -7], 
                    [-60, 90, -7], [-40, 90, -7], [-20, 90, -7], [0, 90, -7], [20, 90, -7], [40, 90, -7], [60, 90, -7], [80, 90, -7]]

        self.batterie=self.bluerov.get_battery_percentage()
        self.batterie_pre=self.batterie
        # definition de l'espace d'action et d'observation
        gym.Env.__init__(self)


        # On a déterminé 5 actions possibles pour le robot : 4 qui gèrent le mode de déplacement (scan, evit, recalibrage et retour base),
        # 1 qui gère la zone à rejoindre avec le mode de déplacement choisis.
        self.action_space =  Box(low=np.array([1,0]), high=np.array([3,143]), dtype=np.float32)

        # self.observation_space = spaces.Dict(spaces=
        #             {
        #                 #X Y Z  depart
        #                 'pos_départ' : spaces.Box(low=-1e6, high=1e6, shape=(1,3), dtype=np.float32),
        #                 #X Y Z arrive
        #                 'intervention' : spaces.Box(low=-1e6, high=1e6, shape=(10,3), dtype=np.float32),
        #                 #X Y Z ma_pos
        #                 'pos_cur' : spaces.Box(low=-1e6, high=1e6, shape=(1,3), dtype=np.float32),
        #                 #Niveau de batterie 
        #                 'nivbat' : spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32),
        #                 #Incert10itude de position
        #                 'pos_error' : spaces.Box(low=0.0, high=1e6, shape=(1,), dtype=np.float32),
        #                 #X Y Z des portes des zones
        #                 'waypnt' : spaces.Box(low=-1e6, high=1e6, shape=(144, 3), dtype=np.float32),
        #                 #data du velodymne, a determiner 
        #                 'data_v' : spaces.Box(low=-1e6, high=1e6, shape=(50, 3), dtype=np.float32),
        #                 # data caméra RGB : il faudra mettre le format de l'image
        #                 'data_cam' : spaces.Box(low=0.0, high=255, shape=(10, 5, 3), dtype=np.int16)
        #             }
        #           )
        self.observation_space = spaces.Box(low=-1e6, high=1e6, shape=(770,), dtype=np.float64)

        # port = '11311'
        # subprocess.Popen(["roscore", "-p", port])

        print("Roscore launched!")

        try:
            rospy.init_node('user_node', log_level=rospy.DEBUG)
            print(colored("INFO : ros node created", 'yellow'))
        except rospy.ROSInterruptException as error:
            print('pubs error with ROS: ', error)
            exit(1)
        

    #Actualisaiton de l'observation 
    def get_observation(self):

        #tirage de l'aléa de baisse subite de la batterie 
        self.batterie=self.bluerov.get_battery_percentage()

        #acquire all obstacles aroundpos_error
        # obstacles = self.bluerov.get_collider_obstacles()
        obstacles = np.ones((50,3)) * 1e5
        true_obstacles = self.bluerov.get_collider_obstacles()
        if np.shape(true_obstacles) != (0,):
            obstacles[0:np.shape(true_obstacles)[0]] = true_obstacles

        #tirage de l'aléa de courant
        self.last_pos_error = self.pos_error
        if self.flag_courant==True:
            self.pos_error=self.pos_error+0.2
            self.facteur_courant=2
        else  :
            self.pos_error=self.pos_error+0.1
            self.facteur_courant=1

        goals = np.ones((10,3)) * 1e6
        goals[0:self.position_goal.shape[0], 0:3] = self.position_goal
        #actualisation des position 
        observations = {
        'pos_départ' : self.position_depart,
        'intervention' : goals,
        'pos_cur' : self.bluerov.get_current_pose(),

        #baisse de la batterie simple
        'nivbat' : np.array([self.batterie]),
        'pos_error' : np.array([self.pos_error]),
        'waypnt' : np.array(self.grid),
        'data_v' : obstacles,
        'data_cam' : np.zeros((10,5,3))
        }

        self.batterie_pre=self.batterie
        # print(observations)
        # for key in observations.keys():
            # observations[key] = observations[key].flatten()
        observations_flat = np.concatenate([obs.flatten() for obs in observations.values()])
        return observations_flat
    
    def reward(self) : 

        score_goal=0
        score_distance=0
        score_battery=0
        score_dep=0
        score_traitement=0
        score_drift=0
       
        # positon_cur= self.bluerov.get_current_pose()
        # positon_goal=np.array(self.position_goal)

        # self.pre_dist_to_goal =  self.current_dist_to_goal

        # #Calcule de la distance du robot par rapport a chaqun des goals 
        # current_dist_to_goal=[]
        # for i in range(0,7) :
        #      current_dist_to_goal.append(np.linalg.norm(positon_goal[i]- positon_cur))

        # #calcule de la ditance de par rapport au départ 
        # current_dist_to_start = np.linalg.norm(self.position_depart - positon_cur)
        
        # for i in range(0,7) :
        #     #On regade si le robot a atteint un goal si oui +100pts et on pase la variable de goal atteint a 1 pour ce goal 
        #     if current_dist_to_goal[i]< GOAL_REACHED_DIST : 
        #         score_goal=100
        #         self.goal_atteint[i]=1

        #     #SI le robot a déja atteint les coordonée de ce goal si alors on calcule rien dautre sinon : 
        #     if self.goal_atteint[i]==0 :
        #         #on calcule du rapprochement/eloignement effectuer par le robot entre le step n et n+1 : 
        #         if abs(current_dist_to_start[i] - self.pre_dist_to_goal[i])>0 :

        #             score_distance=score_distance+abs(current_dist_to_start[i] - self.pre_dist_to_goal[i])*10
        #         #de cette maniere on a pas de maluse quand on s'eloigne d'un goal qu'on vient de traiter.


        # #actualisation de la variable pour le prochain tour de boucle.
        # self.current_dist_to_goal=current_dist_to_goal

        
        # #si batterie  a 0 et le robot pas a la zone de départ alors note de batterie tres mauvaise                
        # if self.batterie==0 :
        #     #si le robot et a moins de 25 mettre de son départ quand 0 alors bien : 
        #     if current_dist_to_start < 25 :
        #         score_batterie=1000 
        #     #sinon mauvais :
        #     else :        
        #         score_batterie = current_dist_to_start*(-100)

        # #verif de la case precendante et de laction précédente 
        # pso_robot=[self.pos_pre[0],self.pos_pre[1]]

        score_drift = -20 * self.pos_error

        #actualisation des variables 
        action_tr= self.cur_action_tr
        action_dep= self.cur_action_dep 

        # #Récupere les données lier a la case dans laquel le robot ce trouve, les diffrentes portes possible et laction a effectuer dans le casse
        type, sortie_disp=box_type_exit(action_dep)

        self.pre_action_tr = action_tr 
        self.pre_action_dep = action_dep

        #Verification du mode de traitement choisi par rapport a la case ou il etait 
        if type=="evit" and self.cur_action_tr == "evit":
            score_traitement=100
        elif type=="scan" and self.cur_action_tr =="evit":
            score_traitement=100
        elif self.cur_action_tr =="return_to_base":
            score_traitement = 100 * self.nb_goals_reached
        elif type=="scan" and self.cur_action_tr == "scan":
            if self.goal_reached:
                score_traitement = 500
                self.goal_reached = False
            else:
                score_traitement = 50
        elif type=="evit" and self.cur_action_tr == "scan":
            score_traitement = -100
        elif self.cur_action_tr == "recalibrate":
            score_traitement = 100 - 10 * self.last_pos_error

        #reward for battery consumation
        score_battery = self.batterie

        # reward agent with its distance from goals
        for x in self.position_goal:
            if x[0]!=1 and x[1] != 1 and x[2] != 1:
                dx = x[0] - self.cur_pos[0]
                dy = x[1] - self.cur_pos[1]
                score_goal += 1/math.hypot(dx, dy)
        score_goal = 100 * score_goal
        
        # reward agent if it chooses the next waypoint next to current position
        if self.check_waypnt_chose ==True:
            score_dep = 100
        else:
            score_dep = 0 

        



        # calcule du score 
        # self.score = score_distance+score_goal+score_traitement+score_dep+score_batterie
        score = score_dep + score_goal + score_battery + score_traitement + score_drift
        return score




    def step(self, action):

        #flag de fin d'épisode 
        done = False
        #nombre de step+1
        self.step_counter += 1
        self.global_step_counter +=1
        print(colored("nb_step = " + str(self.step_counter), "red"))
        print(colored("nb_step_global = " + str(self.global_step_counter), "red"))
        #on recupère les infos contenue dans le dic "action", deux données, mode de traitement et coordonées voulu apres avoir fini le mode de traitement 
        sample_action=action
        self.action_dep, action_trait = round(sample_action[1]), round(sample_action[0])

        self.cur_action_tr = action_trait
        self.cur_action_dep = np.array(self.grid[int(self.action_dep)])

        #position avant de bouger 
        self.pos_pre=self.bluerov.get_current_pose()
        print(colored("Waypoint chosen" + str(self.cur_action_dep), 'red'))
        print(colored("Action_ID : " + str(self.cur_action_tr), 'red')) 
        
        start_action_time = time.time()
        if self.check_waypnt_chosen() == False:
            done = True

        if self.check_waypnt_chosen():
            #mise en route du mode de traitement choisi
            if action_trait == 1 :
                self.cur_action_tr = "evit"
                print(colored("INFO : Obstacles avoidance launched", 'yellow'))
                self.bluerov.do_evit(self.last_waypnt_chosen, self.cur_action_dep)
                self.last_waypnt_chosen = self.cur_action_dep

            elif action_trait == 2 :
                self.cur_action_tr = "scan"
                print(colored("INFO : Scanning launched", 'yellow'))
                self.bluerov.do_scan(self.last_waypnt_chosen, self.cur_action_dep, self.scanning_depth)
                self.last_waypnt_chosen = self.cur_action_dep

            elif  action_trait == 4 :
                self.cur_action_tr = "return_to_base"
                print(colored("INFO : Return to base launched", 'yellow'))
                self.bluerov.do_evit(self.last_waypnt_chosen, self.position_depart)
                self.last_waypnt_chosen = self.cur_action_dep
                done = True
                
            elif  action_trait == 3 :
                self.cur_action_tr = "recalibrate"
                print(colored("INFO : Position recalibration launched", 'yellow'))
                self.bluerov.do_recalibrage(self.pos_pre)
                self.pos_error = 0

        end_action_time = time.time()
        time_action = end_action_time - start_action_time
        self.cur_pos=self.bluerov.get_current_pose()

        self.imponderables() 
        self.calc_battery_loss(time_action)
        self.check_goal_reached()             
        #actualise les infos
        observations = self.get_observation()
        
        #pas capté ce que c'est, mais c'est important 
        info = {}

        #on teste si on a dépassé le nombre de step max d'un épisode
        if self.step_counter >= self._max_episode_length:
            #si oui fin d'épisode
            done = True 

        
        reward=self.reward()
       
        # #verifie si une collision a eu lieu 
        # collision, min_laser = self.observe_collision(self.velodyne_data)

        # #reward negarif si collision + fin d'épisode 
        # if collision:
        #     done = True

       

        # #calculer la distance entre le goal et la position actul 
        # if self.current_dist_to_goal < GOAL_REACHED_DIST or self.cur_pos :
        #     done = True
        
        #imprime le reward dans ros et le nombre de step
        rospy.loginfo(reward)
        rospy.loginfo(self.step_counter)
        
        #envoi les infos a l'IA
        return observations, reward, done, info



    # Reset the state of the environment to an initial state
    def reset(self):

        print(colored("INFO : Reset", 'yellow'))
        
        self.step_counter = 0
        self.position=self.bluerov.get_current_pose()
        self.position_depart=self.change_init()
        self.last_waypnt_chosen = self.position_depart
        print(colored("Init Waypoint = " + str(self.position_depart), 'red'))
        flag_init_sent=False
        while abs(self.position[0] - self.position_depart[0]) > 0.2 and abs(self.position[1] - self.position_depart[1]) > 0.2:
            if flag_init_sent==False:
                desired_position = [self.position_depart[0], self.position_depart[1], -self.position_depart[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.bluerov.set_position_target_local_ned(desired_position)
                flag_init_sent=True
            self.bluerov.update()
            self.position=self.bluerov.get_current_pose()
            self.bluerov.publish()    

        self.position_goal=self.change_goal()
        self.batterie=100
        self.nb_goals_reached = 0
        self.pos_error = 0
        self.last_pos_error = 0
        print(colored("Init_waypnt = " + str(self.position_depart), 'yellow'))
        

        observations = self.get_observation()
        print(colored("INFO : Get_observations passed", 'yellow'))


        return observations
    
    ################### Not DRL methods #######################
    
    # Returns positions of new goals
    def change_goal(self):
        nb_goal = random.randint(5,10)
        goal_possible_areas = [coordonnes_cases[k] for k in range(len(coordonnes_cases)) if coordonnes_cases[k][4]=='scan']
        self.new_goal_areas = random.sample(goal_possible_areas, nb_goal)
        new_goals = np.zeros((nb_goal, 3))
        for i in range(nb_goal):
            x_list = [self.new_goal_areas[i][k][0] for k in range(len(self.new_goal_areas[i])-1)]
            y_list = [coordonnes_cases[i][k][1] for k in range(len(coordonnes_cases[i])-1)]
            new_goals[i, 0], new_goals[i, 1], new_goals[i, 2] = random.randint(min(x_list)+2, max(x_list)-2), random.randint(min(y_list)+2, max(y_list)-2), self.scanning_depth[0]
        return new_goals



        #self.position_goal=waypoint_case(n)

    def change_init(self):
        init_array = np.array([[-70,-60,-7], [-70,-40,-7], [-70,-20,-7], [-70,0,-7], [-70,20,-7], [-70,40,-7], [-70,60,-7], [-70,80,-7],
                               [-60,90,-7], [-40,90,-7], [-20,90,-7], [0,90,-7], [20,90,-7], [40,90,-7], [60,90,-7], [80,90,-7],
                               [90,80,-7], [90,60,-7], [90,40,-7], [90,20,-7], [90,0,-7], [90,-20,-7], [90,-40,-7], [90,-60,-7],
                               [80,-70,-7], [60,-70,-7], [40,-70,-7], [20,-70,-7], [0,-70,-7], [-20,-70,-7], [-40,-70,-7], [-60,-70,-7]])
        n = random.randint(0, len(init_array)-1)
        init_pose = init_array[n]
        return init_pose


    def imponderables(self): 

        #tirage de l'aléa de baisse subite de la batterie 
        if random.randint(1,1000)==1 and self.flag_change_bat==False:
            self.flag_change_bat=True

        else :
            self.batterie_modif=self.batterie

        #tirage de l'aléa de courant
        if random.randint(1,500)==1 and self.flag_courant==False:
            self.flag_courant=True   

    def obstacle_observations(self):
        '''
            Take obstacles data from the bluerov sensor and transform them to match 
            observation space.
        '''
        obstacles = self.bluerov.get_collider_obstacles()
        pass
    
    def check_waypnt_chosen(self):
        x, y, z = self.last_waypnt_chosen[0], self.last_waypnt_chosen[1], self.last_waypnt_chosen[2]
        if self.last_waypnt_chosen[1] % 20 == 0 and self.last_waypnt_chosen[0] % 20 != 0:
            next_waypnt_possible = np.array([[x, y+20, z], [x+10, y+10, z], [x-10, y+10, z],
                                    [x, y-20, z], [x+10, y-10, z], [x-10, y-10, z]])
        else:
            next_waypnt_possible = np.array([[x+20, y, z], [x+10, y+10, z], [x+10, y-10, z],
                                    [x-20, y, z], [x-10, y+10, z], [x-10, y-10, z]])
            
        if (self.cur_action_dep == next_waypnt_possible).all(axis=1).any():
            self.check_waypnt_chose = True
            return True
        else:
            self.check_waypnt_chose = False
            return False
        
    def calc_battery_loss(self, time_action):
        #tirage de l'aléa batterie
        if self.flag_change_bat==True and self.flag_change_bat_init==False  : 
            self.batterie=self.batterie/2
            self.flag_change_bat_init==True 
        if self.flag_change_bat==True and  self.flag_change_bat_init==True : 

            self.batterie -= time_action / 5 * self.facteur_courant

    def check_goal_reached(self):
        x_last, y_last, z_last = self.last_waypnt_chosen[0], self.last_waypnt_chosen[1], self.last_waypnt_chosen[2]
        x_cur, y_cur = self.cur_action_dep[0], self.cur_action_dep[1]
        if abs(x_last-x_cur) == 10:
            area_explored = [[min(x_last, x_cur), min(y_last, y_cur), z_last], [max(x_last, x_cur)+10, min(y_last, y_cur), z_last], [min(x_last, x_cur), max(y_last, y_cur)+10, z_last], [max(x_last, x_cur)+10, max(y_last, y_cur)+10, z_last]]
        elif x_last==x_cur:
            area_explored = [[min(x_last, x_cur)-10, min(y_last, y_cur), z_last], [max(x_last, x_cur)+10, min(y_last, y_cur), z_last], [min(x_last, x_cur)-10, max(y_last, y_cur), z_last], [max(x_last, x_cur)+10, max(y_last, y_cur), z_last]]
        elif y_last==y_last:
            area_explored = [[min(x_last, x_cur), min(y_last, y_cur)-10, z_last], [max(x_last, x_cur), min(y_last, y_cur)-10, z_last], [min(x_last, x_cur), max(y_last, y_cur)+10, z_last], [max(x_last, x_cur), max(y_last, y_cur)+10, z_last]]
        
        for i in range(len(self.new_goal_areas)):
            if area_explored == self.new_goal_areas[i][0:4]:
                self.nb_goals_reached +=1
                self.position_goal[i] = np.ones((1,3))
                self.goal_reached = True

    @staticmethod
    def launch_sitl():

        subprocess.run(["/home/cellule_ia/Desktop/Bluerov_pymavlink/launch_ardu_sitl.sh"], shell=True)





