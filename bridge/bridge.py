#!/usr/bin/env python

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time
import numpy as np

from Gridy_based import Plannification
from dynamic_window_approach import Evitement

from sensor_msgs.msg import LaserScan
import rospy

class Bridge(object):
    """ MAVLink bridge

    Attributes:
        conn (TYPE): MAVLink connection
        data (dict): Deal with all data
    """
    def __init__(self, device='udpin:192.168.2.1:14560', baudrate=115200):
        """
        Args:
            device (str, optional): Input device
                https://ardupilot.github.io/MAVProxy/html/getting_started/starting.html#master
            baudrate (int, optional): Baudrate for serial communication
        """
        self.conn = mavutil.mavlink_connection(device, baud=baudrate)
        

        self.conn.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.conn.target_system, self.conn.target_component))
            
        self.data = {}
        self.current_pose = [0,0,0,0,0,0]
        self.mission_point = 0
        self.mission_point_sent = False
        self.init_evit = False
        self.x_evit = np.array([])
        self.scan = LaserScan()
        self.scan_subscriber= rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.ok_pose = False


    def scan_callback(self, data):
        self.scan = data

    def get_data(self):
        """ Return data

        Returns:
            TYPE: Dict
        """
        return self.data

    def get_all_msgs(self):
        """ Return all mavlink messages

        Returns:
            TYPE: dict
        """
        msgs = []
        while True:
            msg = self.conn.recv_match()
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

    def update(self):
        """ Update data dict
        """
        # Get all messages
        msgs = self.get_all_msgs()
        # Update dict
        for msg in msgs:
            self.data[msg.get_type()] = msg.to_dict()

    def print_data(self):
        """ Debug function, print data dict
        """
        print(self.data)

    def set_mode(self, mode):
        """ Set ROV mode
            http://ardupilot.org/copter/docs/flight-modes.html

        Args:
            mode (str): MMAVLink mode

        Returns:
            TYPE: Description
        """
        mode = mode.upper()
        if mode not in self.conn.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.conn.mode_mapping().keys()))
            return
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.set_mode(mode_id)

    def decode_mode(self, base_mode, custom_mode):
        """ Decode mode from heartbeat
            http://mavlink.org/messages/common#heartbeat

        Args:
            base_mode (TYPE): System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
            custom_mode (TYPE): A bitfield for use for autopilot-specific flags.

        Returns:
            [str, bool]: Type mode string, arm state
        """
        flight_mode = ""

        mode_list = [
            [mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 'MANUAL'],
            [mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED, 'STABILIZE'],
            [mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 'GUIDED'],
            [mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED, 'AUTO'],
            [mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED, 'TEST']
        ]

        if base_mode == 0:
            flight_mode = "PreFlight"
        elif base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            flight_mode = mavutil.mode_mapping_sub[custom_mode]
        else:
            for mode_value, mode_name in mode_list:
                if base_mode & mode_value:
                    flight_mode = mode_name

        arm = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        return flight_mode, arm

    def set_guided_mode(self):
        """ Set guided mode
        """
        #https://github.com/ArduPilot/pymavlink/pull/128
        params = [mavutil.mavlink.MAV_MODE_GUIDED, 0, 0, 0, 0, 0, 0]
        self.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, params)

    def send_command_long(self, command, params=[0, 0, 0, 0, 0, 0, 0], confirmation=0):
        """ Function to abstract long commands

        Args:
            command (mavlink command): Command
            params (list, optional): param1, param2, ..., param7
            confirmation (int, optional): Confirmation value
        """
        self.conn.mav.command_long_send(
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            command,                                # mavlink command
            confirmation,                           # confirmation
            params[0],                              # params
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6]
        )

    # def set_position_target_local_ned(self, param=[]):
    #     """ Create a SET_POSITION_TARGET_LOCAL_NED message
    #         http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED

    #     Args:
    #         param (list, optional): param1, param2, ..., param11
    #     """
    #     if len(param) != 11:
    #         print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')

    #     # Set mask
    #     mask = 0b0000000111111111
    #     # mask = 0b0000000111000000
    #     for i, value in enumerate(param):
    #         if value is not None:
    #             mask -= 1<<i
    #         else:
    #             param[i] = 0.0

    #     #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
    #     self.conn.mav.set_position_target_local_ned_send(
    #         0,                                              # system time in milliseconds
    #         self.conn.target_system,                        # target system
    #         self.conn.target_component,                     # target component
    #         mavutil.mavlink.MAV_FRAME_LOCAL_NED,            # frame
    #         mask,                                           # mask
    #         param[0], param[1], param[2],                   # position x,y,z
    #         param[3], param[4], param[5],                   # velocity x,y,z
    #         param[6], param[7], param[8],                   # accel x,y,z
    #         param[9], param[10])                            # yaw, yaw rate


    def set_position_target_local_ned(self, param=[]):
        """ Create a SET_POSITION_TARGET_LOCAL_NED message
            http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED

        Args:
            param (list, optional): param1, param2, ..., param11
        """
        if len(param) != 11:
            print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')

        

        while not self.is_armed():
            self.conn.arducopter_arm()

        self.conn.set_mode('GUIDED')
        # Set mask
        # mask = 0b0000000111111111
        mask = 0b000111111000
        # for i, value in enumerate(param):
        #     if value is not None:
        #         mask -= 1<<i
        #     else:
        #         param[i] = 0.0


        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        self.conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.conn.target_system, self.conn.target_component,
                           mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(mask),
                            param[0], param[1], param[2],                   # position x,y,z
                            param[3], param[4], param[5],                   # velocity x,y,z
                            param[6], param[7], param[8],                   # accel x,y,z
                            param[9], param[10]))                           # yaw, yaw rate


    def set_attitude_target(self, param=[]):
        """ Create a SET_ATTITUDE_TARGET message
            http://mavlink.org/messages/common#SET_ATTITUDE_TARGET

        Args:
            param (list, optional): param1, param2, ..., param7
        """
        if len(param) != 8:
            print('SET_ATTITUDE_TARGET need 8 params')

        # Set mask
        mask = 0b11111111
        for i, value in enumerate(param[4:-1]):
            if value is not None:
                mask -= 1<<i
            else:
                param[i+3] = 0.0

        if param[7] is not None:
            mask += 1<<6
        else:
            param[7] = 0.0

        q = param[:4]

        if q != [None, None, None, None]:
            mask += 1<<7
        else:
            q = [1.0, 0.0, 0.0, 0.0]

        self.conn.mav.set_attitude_target_send(0,   # system time in milliseconds
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            mask,                                   # mask
            q,                                      # quaternion attitude
            param[4],                               # body roll rate
            param[5],                               # body pitch rate
            param[6],                               # body yaw rate
            param[7])                               # thrust

    def set_servo_pwm(self, id, pwm=1500):
        """ Set servo pwm

        Args:
            id (int): Servo id
            pwm (int, optional): pwm value 1100-2000
        """

        #http://mavlink.org/messages/common#MAV_CMD_DO_SET_SERVO
        # servo id
        # pwm 1000-2000
        mavutil.mavfile.set_servo(self.conn, id, pwm)

    def set_rc_channel_pwm(self, id, pwm=1500):
        """ Set RC channel pwm value

        Args:
            id (TYPE): Channel id
            pwm (int, optional): Channel pwm value 1100-2000
        """
        rc_channel_values = [65535 for _ in range(8)] #8 for mavlink1
        rc_channel_values[id] = pwm
        #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,                # target_system
            self.conn.target_component,             # target_component
            *rc_channel_values)                     # RC channel list, in microseconds.
    
    def set_manual_control(self,joy_list=[0]*4, buttons_list=[0]*16):
        """ Set a MANUAL_CONTROL message for dealing with more control with ArduSub
        for now it is just to deal with lights under test...
        """
        x,y,z,r = 0,0,0,0#32767,32767,32767,32767
        b = 0
        for i in range(len(buttons_list)):
            b = b | (buttons_list[i]<<i)
        print("MANUAL_CONTROL_SEND : x : {}, y : {}, z : {}, r : {}, b : {}".format(x,y,z,r,b))
        #https://mavlink.io/en/messages/common.html MANUAL_CONTROL ( #69 )
        self.conn.mav.manual_control_send(
               self.conn.target_system,
                x,
                y,
                z,
                r,
                b)


    def arm_throttle(self, arm_throttle):
        """ Arm throttle

        Args:
            arm_throttle (bool): Arm state
        """
        if arm_throttle:
            self.conn.arducopter_arm()
        else:
            #http://mavlink.org/messages/common#MAV_CMD_COMPONENT_ARM_DISARM
            # param1 (0 to indicate disarm)
            # Reserved (all remaining params)
            self.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [0, 0, 0, 0, 0, 0, 0]
            )



    def set_target_depth(self,depth):

        self.conn.set_mode('ALT_HOLD')

        while not self.is_armed():
            self.conn.arducopter_arm()

        print('Bluerov is armed')

        self.conn.mav.set_position_target_global_int_send(
            0,     
            0, 0,   
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT, # frame
            0b0000111111111000,
            0,0, depth,
            0 , 0 , 0 , # x , y , z velocity in m/ s ( not used )
            0 , 0 , 0 , # x , y , z acceleration ( not supported yet , ignored in GCS Mavlink )
            0 , 0 ) # yaw , yawrate ( not supported yet , ignored in GCS Mavlink )

        print('set_position_target_global_int_send')    

    def is_armed(self):
        try:
            return bool(self.conn.wait_heartbeat().base_mode & 0b10000000)
        except:
            return False  


    def set_target_attitude(self, roll, pitch, yaw, control_yaw=True):
        bitmask = (1<<6 | 1<<3)  if control_yaw else 1<<6

        self.conn.mav.set_attitude_target_send(
            0,     
            0, 0,   
            bitmask,
            QuaternionBase([math.radians(roll), math.radians(pitch), math.radians(yaw)]), # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            0, #roll rate
            0, #pitch rate
            0, 0)    # yaw rate, thrust 

    def get_bluerov_position_data(self): #Loop comunicazione con QGROUND
        # Get some information !
        self.update()
        if 'LOCAL_POSITION_NED' in self.get_data():              
            
            local_position_data = self.get_data()['LOCAL_POSITION_NED']
            xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
            self.current_pose[0:3] = [xyz_data[0], xyz_data[1], xyz_data[2]]
            # print(xyz_data)



        # waiting for 2 seconds after writing
        # the file
        # else:
        #     print("no local position ned")
        # time.sleep(2)
        # print("Finished background file write to",
        #                                  self.out)  



    def do_scan(self, px, py, oz):

        desired_position = [0.0, 0.0, -oz[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        current_pose = self.current_pose
        desired_position[0], desired_position[1] = px[self.mission_point], py[self.mission_point]

        if abs(current_pose[0] - desired_position[0]) < 0.2 and abs(current_pose[1] - desired_position[1]) < 0.2:
            print("Arrivé au point")
            if self.mission_point == len(px):
                self.ok_pose = True
            self.mission_point += 1
            desired_position[0], desired_position[1] = px[self.mission_point], py[self.mission_point]
            self.mission_point_sent = False

        if self.mission_point_sent == False:
            time.sleep(0.05)
            # self.ok_pose = False
            self.set_position_target_local_ned(desired_position)
            time.sleep(0.05)
            self.mission_point_sent = True

        if self.ok_pose == True :
            print('Mission terminée')
            self.mission_point = 0
            self.mission_point_sent = False

        
    def do_evit(self, evitement, x_init, goal):

        z_mission = x_init[2]

        if self.init_evit == False:

            if self.mission_point_sent == False:
                time.sleep(0.05)
                self.ok_pose = False
                initial_position = [x_init[0], x_init[1], -z_mission, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.set_position_target_local_ned(initial_position)
                time.sleep(0.05)
                self.mission_point_sent = True

            if abs(self.current_pose[0] - x_init[0]) < 0.2 and abs(self.current_pose[1] - x_init[1]) < 0.2:
                self.init_evit = True
                print("je suis au point initial")
                self.x = np.array([x_init[0], x_init[1], math.pi / 8.0, 0.0, 0.0])
                self.mission_point_sent = False



        
        ob = self.get_obstacle(20)

        if self.init_evit == True:
            # print("init_evit " + str(self.init_evit))
            if abs(self.current_pose[0] - self.x[0]) < 0.02 and abs(self.current_pose[1] - self.x[1]) < 0.02:
                # print("avant do evit planning")
                current_pose = np.array([self.current_pose[0], self.current_pose[1], self.x[2], self.x[3], self.x[4]])
                print("entrée = " + str(self.x))
                self.x = evitement.planning(ob, self.x)
                self.mission_point_sent = False
                print("sortie = " + str(self.x))
                # print("self.current_pose = " + str(self.current_pose))

            if self.mission_point_sent == False:
                time.sleep(0.05)
                self.ok_pose = False
                desired_position = [self.x[0], self.x[1], -z_mission, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.x[3], self.x[4]]
                self.set_position_target_local_ned(desired_position)
                time.sleep(0.05)
                self.mission_point_sent = True


        



    # def get_obstacle(self, decoupage):
    #     scan_range = []
    #     """
    #         Pour un LaserScan avec un champ de visions de (-80,80)deg et 540 points
    #     """

    #     # scan = None
    #     # while scan is None:
    #     #     try:
    #     #         scan = rospy.wait_for_message('/scan', LaserScan)
    #     #     except:
    #     #         pass

    #     for i in range(len(self.scan.ranges)):
    #         if self.scan.ranges[i] == float('Inf'):
    #             scan_range.append(10)
    #         elif np.isnan(self.scan.ranges[i]):
    #             scan_range.append(0)
    #         else:
    #             scan_range.append(self.scan.ranges[i])

    #     obstacle_min_range = round(min(scan_range), 2)
    #     obstacle_angle = np.argmin(scan_range)

    #     obstacle_x = obstacle_min_range * math.cos(obstacle_angle)
    #     obstacle_y = obstacle_min_range * math.sin(obstacle_angle)
    #     # if min_range > min(scan_range) > 0:
    #     #     done = True

    #     if obstacle_x < 0.15 and obstacle_y < 0.15 :
    #         return np.array([[]])


    #     obstacle = np.array([[obstacle_x + self.current_pose[0], obstacle_y + self.current_pose[1]]])
    #     # current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
    #     # if current_distance < 0.2:
    #     #     self.get_goalbox = True

    #     return obstacle


    def get_obstacle(self, decoupage):
        obstacle =  np.zeros((decoupage,2))
        """
            Pour un LaserScan avec un champ de visions de (-80,80)deg et 540 points
        """
        scan_increments = len(self.scan.ranges)
        # scan = None
        # while scan is None:
        #     try:
        #         scan = rospy.wait_for_message('/scan', LaserScan)
        #     except:
        #         pass
        scan_zone_flag = 1
        min_range = []

        for i in range(scan_increments):

            if self.scan.ranges[i] != 0:
                min_range.append(self.scan.ranges[i])

            if i == scan_zone_flag * (scan_increments / decoupage) -1:
                if len(min_range) == 0:
                    obstacle[scan_zone_flag - 1, 0], obstacle[scan_zone_flag - 1, 1] = float("Inf"), float("Inf")
                else:
                    obstacle_min_range = round(min(min_range), 2)
                    obstacle_angle = np.argmin(min_range)
                    obstacle_x = obstacle_min_range * math.cos(obstacle_angle)
                    obstacle_y = obstacle_min_range * math.sin(obstacle_angle)
                    obstacle[scan_zone_flag - 1, 0], obstacle[scan_zone_flag - 1, 1] = obstacle_x + self.current_pose[0], obstacle_y + self.current_pose[1]
                min_range = []
                scan_zone_flag += 1

        # if min_range > min(scan_range) > 0:
        #     done = True

        # current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        # if current_distance < 0.2:
        #     self.get_goalbox = True

        return obstacle


    def loop_control(self):
	

        #print('ok')
        ok_z=False
        ok_yaw=False
        ok_xy=False
        #yaw_temp=100
        self.conn.set_mode('GUIDED')


        while(1):
            while not self.is_armed():
                self.conn.arducopter_arm()
        #leggo i miei dati
            x_rov=self.current_pose[0]
            y_rov=self.current_pose[1]
            z_rov=self.current_pose[2]
            roll_rov=self.current_pose[3]
            pitch_rov=self.current_pose[4]
            yaw_rov=math.degrees(self.current_pose[5])
            # if yaw_rov<0:
            #     yaw_ok=yaw_rov+360
            # else:
            #     yaw_ok=yaw_rov



        
    #		diff_x=u_rovpos[0]-z_utm_des[0]
    #		diff_y=u_rovpos[1]-z_utm_des[1]
    #		dist = sqrt((diff_x)^2+(diff_y)^2)

            #yaw_path=degrees(atan2(diff_y,diff_x))
            # yaw_path=330

            # z_utm_des=desideredPosition()		
            
            # diff_z= z_utm_des[2] + z_rov
            # #print(z_utm_des[2],diff_z,z_rov)
            # if abs(diff_z) <= 0.2:
            #     ok_z=True
            #     print('ok depth')
            # else:
            #     ok_z=False
            #     attitudeControl.set_target_depth(z_utm_des[2],master)
            
            # #print(z_rov,yaw_rov)
            # if ok_z == True and ok_yaw==False:
            #     attitudeControl.set_target_attitude(master,0, 0, yaw_path)
            
            # diff_yaw= yaw_path - yaw_ok
                
            # if abs(diff_yaw) <= 5:
            #     ok_yaw=True
            #     print('yaw_ok')
            # else:
            #     ok_yaw=False

            # time.sleep(0.01)
            
            # if ok_z== True and ok_yaw==True and ok_xy==False:
            #     print('avanza')
            #     #u_x=pidRov.main(dist)
            #     master.mav.manual_control_send( #x - y- z - r - button 
            #     master.target_system, #x,y and r will be between [-1000 and 1000].
            #     500, # x, da sostituire con un pid . 
            #     0, #y
            #     500, #la Z va da -1000 a 1000, quindi 500 è ferma
            #     0, #r è lo yaw
            #     0)
            # else:
            #     master.mav.manual_control_send( #x - y- z - r - button 
            #     master.target_system, #x,y and r will be between [-1000 and 1000].
            #     0, # x, da sostituire con un pid . 
            #     0, #y
            #     500, #la Z va da -1000 a 1000, quindi 500 è ferma
            #     0, #r è lo yaw
      


if __name__ == '__main__':
    bridge = Bridge()
    #bridge = Bridge(device='udp:localhost:14550')
    #i=0
    #filemav = open("mavlinkdata.txt", 'w')
    while True:
        bridge.update()
        bridge.print_data()
        #filemav.write("{}\n".format(bridge.data))
        #bridge.set_servo_pwm(9,1800)
        #i+=1
    #filemav.close()
        


#        if 'SCALED_PRESSURE' not in bridge.get_data():
#            print('NO PRESSURE DATA')


#        else :
#            bar30_data = bridge.get_data()['SCALED_PRESSURE']
#            print("bar30data : ",bar30_data)
#            time_boot_ms = bar30_data['time_boot_ms']
#            press_abs    = bar30_data['press_abs']
#            press_diff   = bar30_data['press_diff']
#            temperature  = bar30_data['temperature']
#            print("\n\n\n")
#            print( "time :",time_boot_ms,"press_abs :", press_abs, "press_diff :",press_diff, "temperature :", temperature)