import numpy as np
import random
import math

class GroundState:
    def __init__(self, nb_goals=5) -> None:
        self.grid_waypoints =[[-60, -70, 6], [-40, -70, 6], [-20, -70, 6], [0, -70, 6], [20, -70, 6], [40, -70, 6], [60, -70, 6], [80, -70, 6], 
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
        self.corners_n_actions= [[[-70, -70, 6], [-50, -70, 6], [-70, -50, 6], [-50, -50, 6], 'scan'], 
                                [[-50, -70, 6], [-30, -70, 6], [-50, -50, 6], [-30, -50, 6], 'evit'], 
                                [[-30, -70, 6], [-10, -70, 6], [-30, -50, 6], [-10, -50, 6], 'scan'],
                                [[-10, -70, 6], [10, -70, 6], [-10, -50, 6], [10, -50, 6], 'evit'],
                                [[10, -70, 6], [30, -70, 6], [10, -50, 6], [30, -50, 6], 'scan'], 
                                [[30, -70, 6], [50, -70, 6], [30, -50, 6], [50, -50, 6], 'evit'], 
                                [[50, -70, 6], [70, -70, 6], [50, -50, 6], [70, -50, 6], 'scan'], 
                                [[70, -70, 6], [90, -70, 6], [70, -50, 6], [90, -50, 6], 'scan'], 
                                [[-70, -50, 6], [-50, -50, 6], [-70, -30, 6], [-50, -30, 6], 'scan'], 
                                [[-50, -50, 6], [-30, -50, 6], [-50, -30, 6], [-30, -30, 6], 'evit'], 
                                [[-30, -50, 6], [-10, -50, 6], [-30, -30, 6], [-10, -30, 6], 'scan'], 
                                [[-10, -50, 6], [10, -50, 6], [-10, -30, 6], [10, -30, 6], 'scan'], 
                                [[10, -50, 6], [30, -50, 6], [10, -30, 6], [30, -30, 6], 'scan'], 
                                [[30, -50, 6], [50, -50, 6], [30, -30, 6], [50, -30, 6], 'evit'], 
                                [[50, -50, 6], [70, -50, 6], [50, -30, 6], [70, -30, 6], 'scan'], 
                                [[70, -50, 6], [90, -50, 6], [70, -30, 6], [90, -30, 6], 'scan'], 
                                [[-70, -30, 6], [-50, -30, 6], [-70, -10, 6], [-50, -10, 6], 'scan'], 
                                [[-50, -30, 6], [-30, -30, 6], [-50, -10, 6], [-30, -10, 6], 'scan'], 
                                [[-30, -30, 6], [-10, -30, 6], [-30, -10, 6], [-10, -10, 6], 'evit'], 
                                [[-10, -30, 6], [10, -30, 6], [-10, -10, 6], [10, -10, 6], 'scan'], 
                                [[10, -30, 6], [30, -30, 6], [10, -10, 6], [30, -10, 6], 'evit'], 
                                [[30, -30, 6], [50, -30, 6], [30, -10, 6], [50, -10, 6], 'scan'], 
                                [[50, -30, 6], [70, -30, 6], [50, -10, 6], [70, -10, 6], 'evit'], 
                                [[70, -30, 6], [90, -30, 6], [70, -10, 6], [90, -10, 6], 'evit'], 
                                [[-70, -10, 6], [-50, -10, 6], [-70, 10, 6], [-50, 10, 6], 'evit'], 
                                [[-50, -10, 6], [-30, -10, 6], [-50, 10, 6], [-30, 10, 6], 'evit'], 
                                [[-30, -10, 6], [-10, -10, 6], [-30, 10, 6], [-10, 10, 6], 'evit'], 
                                [[-10, -10, 6], [10, -10, 6], [-10, 10, 6], [10, 10, 6], 'evit'], 
                                [[10, -10, 6], [30, -10, 6], [10, 10, 6], [30, 10, 6], 'scan'], 
                                [[30, -10, 6], [50, -10, 6], [30, 10, 6], [50, 10, 6], 'evit'], 
                                [[50, -10, 6], [70, -10, 6], [50, 10, 6], [70, 10, 6], 'scan'], 
                                [[70, -10, 6], [90, -10, 6], [70, 10, 6], [90, 10, 6], 'scan'], 
                                [[-70, 10, 6], [-50, 10, 6], [-70, 30, 6], [-50, 30, 6], 'scan'], 
                                [[-50, 10, 6], [-30, 10, 6], [-50, 30, 6], [-30, 30, 6], 'scan'], 
                                [[-30, 10, 6], [-10, 10, 6], [-30, 30, 6], [-10, 30, 6], 'evit'], 
                                [[-10, 10, 6], [10, 10, 6], [-10, 30, 6], [10, 30, 6], 'scan'], 
                                [[10, 10, 6], [30, 10, 6], [10, 30, 6], [30, 30, 6], 'evit'], 
                                [[30, 10, 6], [50, 10, 6], [30, 30, 6], [50, 30, 6], 'scan'], 
                                [[50, 10, 6], [70, 10, 6], [50, 30, 6], [70, 30, 6], 'evit'], 
                                [[70, 10, 6], [90, 10, 6], [70, 30, 6], [90, 30, 6], 'scan'], 
                                [[-70, 30, 6], [-50, 30, 6], [-70, 50, 6], [-50, 50, 6], 'scan'],
                                [[-50, 30, 6], [-30, 30, 6], [-50, 50, 6], [-30, 50, 6], 'evit'], 
                                [[-30, 30, 6], [-10, 30, 6], [-30, 50, 6], [-10, 50, 6], 'scan'], 
                                [[-10, 30, 6], [10, 30, 6], [-10, 50, 6], [10, 50, 6], 'evit'], 
                                [[10, 30, 6], [30, 30, 6], [10, 50, 6], [30, 50, 6], 'scan'], 
                                [[30, 30, 6], [50, 30, 6], [30, 50, 6], [50, 50, 6], 'evit'], 
                                [[50, 30, 6], [70, 30, 6], [50, 50, 6], [70, 50, 6], 'evit'], 
                                [[70, 30, 6], [90, 30, 6], [70, 50, 6], [90, 50, 6], 'evit'], 
                                [[-70, 50, 6], [-50, 50, 6], [-70, 70, 6], [-50, 70, 6], 'evit'], 
                                [[-50, 50, 6], [-30, 50, 6], [-50, 70, 6], [-30, 70, 6], 'scan'], 
                                [[-30, 50, 6], [-10, 50, 6], [-30, 70, 6], [-10, 70, 6], 'evit'], 
                                [[-10, 50, 6], [10, 50, 6], [-10, 70, 6], [10, 70, 6], 'scan'],
                                [[10, 50, 6], [30, 50, 6], [10, 70, 6], [30, 70, 6], 'evit'], 
                                [[30, 50, 6], [50, 50, 6], [30, 70, 6], [50, 70, 6], 'scan'], 
                                [[50, 50, 6], [70, 50, 6], [50, 70, 6], [70, 70, 6], 'evit'], 
                                [[70, 50, 6], [90, 50, 6], [70, 70, 6], [90, 70, 6], 'evit'], 
                                [[-70, 70, 6], [-50, 70, 6], [-70, 90, 6], [-50, 90, 6], 'evit'],
                                [[-50, 70, 6], [-30, 70, 6], [-50, 90, 6], [-30, 90, 6], 'scan'], 
                                [[-30, 70, 6], [-10, 70, 6], [-30, 90, 6], [-10, 90, 6], 'scan'], 
                                [[-10, 70, 6], [10, 70, 6], [-10, 90, 6], [10, 90, 6], 'evit'],
                                [[10, 70, 6], [30, 70, 6], [10, 90, 6], [30, 90, 6], 'scan'],
                                [[30, 70, 6], [50, 70, 6], [30, 90, 6], [50, 90, 6], 'scan'],
                                [[50, 70, 6], [70, 70, 6], [50, 90, 6], [70, 90, 6], 'evit'],
                                [[70, 70, 6], [90, 70, 6], [70, 90, 6], [90, 90, 6], 'evit']]
        self.init_array = np.array([[-70,-60,6], [-70,-40,6], [-70,-20,6], [-70,0,6], [-70,20,6], [60,40,6], [-70,60,6], [-70,80,6],
                               [-60,90,6], [-40,90,6], [-20,90,6], [0,90,6], [20,90,6], [40,90,6], [60,90,6], [80,90,6],
                               [90,80,6], [90,60,6], [90,40,6], [90,20,6], [90,0,6], [90,-20,6], [90,-40,6], [90,-60,6],
                               [80,-70,6], [60,-70,6], [40,-70,6], [20,-70,6], [0,-70,6], [-20,-70,6], [-40,-70,6], [-60,-70,6]])
        self.goals = np.zeros((nb_goals,4))
        self.nb_goals = nb_goals
        self.init_pose = self.reset_init_position()
        self.last_area = []
        self.__x_min = -70
        self.__x_max = 90
        self.__y_min = -70
        self.__y_max = 90
        self.__depth_min = -8
        self.__depth_max = 0
        
    # def action_to_waypoint(self, action, current_rov_position):
    #     waypoint = np.array(self.__possible_next_waypnts(current_rov_position)[int(action)])
    #     return waypoint
    
    def action_to_waypoint(self, action, current_rov_position, current_position_state):
        x, y, z = current_rov_position[0], current_rov_position[1], current_rov_position[2]
        actions_from_row = [0, 1, 3, 4, 5, 7]
        actions_from_col = [1, 2, 3, 5, 6, 7]
        actions_from_state = [actions_from_row, actions_from_col]

        # "North", "North-Est", "Est", "South-Est", "South", "South-West", "West", "North-West"
        movement_coords = [[x, y+20, z],[x+10, y+10, z], [x+20, y, z], [x+10, y-10, z], [x, y-20, z], [x-10, y-10, z], [x-20, y, z], [x-10, y+10, z]]
        if action in actions_from_state[current_position_state]:
            new_waypnt = np.array(movement_coords[int(action)])
        else:
            new_waypnt = None
        if type(new_waypnt) != type(None) and self.is_wpnt_inbound(new_waypnt) == False:
            new_waypnt = None
        return new_waypnt
    
    def wpnt_is_raw_or_col(self, current_rov_position):
        """
        Returns 0 if current position is on a raw, and 1 if it is on a column.
        """
        x, y = current_rov_position[0], current_rov_position[1]
        if x % 20 == 0 and y % 20 != 0:
            return 0
        else:
            return 1

    def is_wpnt_inbound(self, coordinates):
        if [coordinates[0], coordinates[1], coordinates[2]] in self.grid_waypoints:
             return True
        else:
             return False
        
    def get_choice_of_action(self, current_rov_position):
        choice_of_action = [0, 0, 0, 0, 0, 0, 0, 0]
        row_or_col = self.wpnt_is_raw_or_col(current_rov_position)
        x, y, z = current_rov_position[0], current_rov_position[1], current_rov_position[2]
        movement_coords = [[x, y+20, z],[x+10, y+10, z], [x+20, y, z], [x+10, y-10, z], [x, y-20, z], [x-10, y-10, z], [x-20, y, z], [x-10, y+10, z]]
        actions_from_row = [0, 1, 3, 4, 5, 7]
        actions_from_col = [1, 2, 3, 5, 6, 7]
        actions_from_state = [actions_from_row, actions_from_col]
        for i in range(len(movement_coords)):
            if i in actions_from_state[row_or_col] and self.is_wpnt_inbound(movement_coords[i]):
                choice_of_action[i] = 1

        return choice_of_action, row_or_col
        
    def reset_goals(self):
        self.goals[:,0:3] = self.__define_new_goals()
        self.goals[:,3] = 0
        return self.goals
    
    def reset_init_position(self):
        n = random.randint(0, len(self.init_array)-1)
        self.init_pose = self.init_array[n]
        return self.init_pose
        
    def update_goal_reached(self, current_position, last_position):
        current_area = self.__identify_area(current_position, last_position)
        goal_idx = self.__check_goal_reached(current_area)
        if goal_idx == -1:
            return self.goals
        else:
            self.goals[goal_idx,3] = 1
            return self.goals
    
    def get_parameters(self):
        return self.__x_min, self.__x_max, self.__y_min, self.__y_max, self.__depth_min, self.__depth_max
    
    def get_distance_to_base(self, current_rov_position):
        distance_to_base = self.__calculate_distance_to_base(current_rov_position)
        return distance_to_base

    def __check_goal_reached(self, current_area):
        current_area = np.array(current_area)
        x_min, x_max = min(current_area[:,0]), max(current_area[:,0])
        y_min, y_max = min(current_area[:,1]), max(current_area[:,1])
        for i in range(len(self.goals)):
            if x_min <= self.goals[i,0] <= x_max and y_min <= self.goals[i,1] <= y_max :
                return i
        return -1

    def __identify_area(self, current_position, last_position):
        current_area = self.last_area
        x_last, y_last, z_last = last_position[0], last_position[1], last_position[2]
        x_cur, y_cur = current_position[0], current_position[1]
        if y_last-y_cur == 10:
            if x_cur % 20 == 0:
                current_area = [[x_cur+10, y_cur, z_last], [x_cur-10, y_cur, z_last], 
                                [x_cur+10, y_cur+20, z_last], [x_cur-10, y_cur+20, z_last]]
            else:
                current_area = [[x_last+10, y_last, z_last], [x_last-10, y_last, z_last], 
                                [x_last+10, y_last+20, z_last], [x_last-10, y_last+20, z_last]]
                
        elif y_last-y_cur == -10:
            if x_cur % 20 == 0:
                current_area = [[x_cur+10, y_cur, z_last], [x_cur-10, y_cur, z_last], 
                                [x_cur+10, y_cur-20, z_last], [x_cur-10, y_cur-20, z_last]]
            else:
                current_area = [[x_last+10, y_last, z_last], [x_last-10, y_last, z_last], 
                                [x_last+10, y_last-20, z_last], [x_last-10, y_last-20, z_last]]
                
        elif x_last==x_cur:
            current_area = [[min(x_last, x_cur)-10, min(y_last, y_cur), z_last], [max(x_last, x_cur)+10, min(y_last, y_cur), z_last], [min(x_last, x_cur)-10, max(y_last, y_cur), z_last], 
                             [max(x_last, x_cur)+10, max(y_last, y_cur), z_last]]
        elif y_last==y_cur:
            current_area = [[min(x_last, x_cur), min(y_last, y_cur)-10, z_last], [max(x_last, x_cur), min(y_last, y_cur)-10, z_last], 
                             [min(x_last, x_cur), max(y_last, y_cur)+10, z_last], [max(x_last, x_cur), max(y_last, y_cur)+10, z_last]]
            
        self.last_area = current_area

        return current_area
    
    def __define_new_goals(self):
        goal_possible_areas = [self.corners_n_actions[k] for k in range(len(self.corners_n_actions)) if self.corners_n_actions[k][4]=='scan']
        self.goal_areas = random.sample(goal_possible_areas, self.nb_goals)
        new_goals = np.zeros((self.nb_goals, 3))
        for i in range(self.nb_goals):
            x_list = [self.goal_areas[i][k][0] for k in range(len(self.goal_areas[i])-1)]
            y_list = [self.goal_areas[i][k][1] for k in range(len(self.goal_areas[i])-1)]
            new_goals[i, 0], new_goals[i, 1], new_goals[i, 2] = random.randint(min(x_list)+2, max(x_list)-2), random.randint(min(y_list)+2, max(y_list)-2), self.goal_areas[i][0][2]
        return new_goals
    
    def __possible_next_waypnts(self, current_waypoint):
        x, y, z = current_waypoint[0], current_waypoint[1], current_waypoint[2]
        if x % 20 == 0 and y % 20 != 0:
            next_waypnts_possible = np.array([[x, y+20, z], [x+10, y+10, z], [x-10, y+10, z],
                                    [x, y-20, z], [x+10, y-10, z], [x-10, y-10, z]])
        else:
            next_waypnts_possible = np.array([[x+20, y, z], [x+10, y+10, z], [x+10, y-10, z],
                                    [x-20, y, z], [x-10, y+10, z], [x-10, y-10, z]])
        return next_waypnts_possible
    
    def __calculate_distance_to_base(self, current_position):
        dx = current_position[0] - self.init_pose[0]
        dy = current_position[1] - self.init_pose[1]
        distance_to_base = math.hypot(dx, dy)
        return distance_to_base


ground = GroundState(5)
ground.goals = np.array([[60, -67, 6, 0],
                         [-54, -58, 6, 0.],
                         [-43, -12, 6, 0],
                         [-57, 12, 6, 0],
                         [2, 27, 6, 0]])
current_area = ground.update_goal_reached(np.array([60, -70, 6]), np.array([60, -50, 6]))
liste, row_or_col = ground.get_choice_of_action(np.array([40,-50,6]))
# row_or_col = ground.wpnt_is_raw_or_col(np.array([40,-70,6]))
new_wpnt = ground.action_to_waypoint(3, np.array([40, -50, 6]), row_or_col)
print(new_wpnt)
print(liste)