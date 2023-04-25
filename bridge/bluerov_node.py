#!/usr/bin/env python

from __future__ import division

import json
import math
import re
import rospy
import sys
import time
from matplotlib import pyplot

from bridge import Bridge

import threading

from Gridy_based import Plannification
from dynamic_window_approach import Evitement
from sensor_msgs.msg import LaserScan
import attitude_control

try:
    from pubs import Pubs
#     from subs import Subs
#     from video import Video
except:
    from bluerov.pubs import Pubs
#     from bluerov.subs import Subs
#     from bluerov.video import Video

# from TrajectoryGenerator import TrajectoryGenerator

# convert opencv image to ros image msg
from cv_bridge import CvBridge

# msgs type
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
# from bluerov_ros_playground.msg import Bar30
# from bluerov_ros_playground.msg import Attitude 
# from bluerov_ros_playground.msg import State 

"""
Generates a quintic polynomial trajectory.
Author: Daniel Ingram (daniel-s-ingram)
"""

import numpy as np

class TrajectoryGenerator():
    def __init__(self, start_pos, des_pos, T, start_vel=[0,0,0], des_vel=[0,0,0], start_acc=[0,0,0], des_acc=[0,0,0]):
        self.start_x = start_pos[0]
        self.start_y = start_pos[1]
        self.start_z = start_pos[2]

        self.des_x = des_pos[0]
        self.des_y = des_pos[1]
        self.des_z = des_pos[2]

        self.start_x_vel = start_vel[0]
        self.start_y_vel = start_vel[1]
        self.start_z_vel = start_vel[2]

        self.des_x_vel = des_vel[0]
        self.des_y_vel = des_vel[1]
        self.des_z_vel = des_vel[2]

        self.start_x_acc = start_acc[0]
        self.start_y_acc = start_acc[1]
        self.start_z_acc = start_acc[2]

        self.des_x_acc = des_acc[0]
        self.des_y_acc = des_acc[1]
        self.des_z_acc = des_acc[2]

        self.T = T

    def solve(self):
        A = np.array(
            [[0, 0, 0, 0, 0, 1],
             [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
             [0, 0, 0, 0, 1, 0],
             [5*self.T**4, 4*self.T**3, 3*self.T**2, 2*self.T, 1, 0],
             [0, 0, 0, 2, 0, 0],
             [20*self.T**3, 12*self.T**2, 6*self.T, 2, 0, 0]
            ])

        b_x = np.array(
            [[self.start_x],
             [self.des_x],
             [self.start_x_vel],
             [self.des_x_vel],
             [self.start_x_acc],
             [self.des_x_acc]
            ])
        
        b_y = np.array(
            [[self.start_y],
             [self.des_y],
             [self.start_y_vel],
             [self.des_y_vel],
             [self.start_y_acc],
             [self.des_y_acc]
            ])

        b_z = np.array(
            [[self.start_z],
             [self.des_z],
             [self.start_z_vel],
             [self.des_z_vel],
             [self.start_z_acc],
             [self.des_z_acc]
            ])

        self.x_c = np.linalg.solve(A, b_x)
        self.y_c = np.linalg.solve(A, b_y)
        self.z_c = np.linalg.solve(A, b_z)


class BlueRov(Bridge):
    def __init__(self, device='udp:192.168.2.1:14550', baudrate=115200):
        """ BlueRov ROS Bridge

        Args:
            device (str, optional): mavproxy device description
            baudrate (int, optional): Serial baudrate
        """
        super(BlueRov, self).__init__(device, baudrate)
        self.pub = Pubs()
        # self.sub = Subs()
        self.ROV_name = 'BlueRov2'
        self.model_base_link = '/base_link'

        # # self.video = Video()
        # self.video_bridge = CvBridge()

        self.pub_topics = [
            [
                self._create_position_msg,
                '/local_position',
                PoseStamped,
                1
            ],
            # [
            #     self._create_battery_msg,
            #     '/battery',
            #     BatteryState,
            #     1
            # ],
        #     [
        #         self._create_camera_msg,
        #         '/camera/image_raw',
        #         Image,
        #         1
        #     ],
        #     # [
        #     #     self._create_ROV_state,
        #     #     '/state',
        #     #     State,
        #     #     1
        #     # ],
        #     [
        #         self._create_imu_msg,
        #         '/imu/data',
        #         Imu,
        #         1
        #     ],
            [
                self._create_odometry_msg,
                '/odometry',
                Odometry,
                1
            ]
        #     # [
        #     #     self._create_bar30_msg,
        #     #     '/bar30',
        #     #     Bar30,
        #     #     1
        #     # ],
        #     # [
        #     #     self._create_imu_euler_msg,
        #     #     '/imu/attitude',
        #     #     Attitude,
        #     #     1
        #     # ]
        ]

        # self.sub_topics= [
        #     [
        #         self._setpoint_velocity_cmd_vel_callback,
        #         '/setpoint_velocity/cmd_vel',
        #         TwistStamped,
        #         1
        #     ],
        #     [
        #         self._set_servo_callback,
        #         '/servo{}/set_pwm',
        #         UInt16,
        #         1,
        #         [1, 2, 3, 4, 5, 6, 7, 8]
        #     ],
        #     [
        #         self._set_rc_channel_callback,
        #         '/rc_channel{}/set_pwm',
        #         UInt16,
        #         1,
        #         [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        #     ],
        #     [
        #         self._set_mode_callback,
        #         '/mode/set',
        #         String,
        #         1
        #     ],
        #     [
        #         self._arm_callback,
        #         '/arm',
        #         Bool,
        #         1
        #     ],
        #     [
        #         self._manual_control_callback,
        #         '/manual_control',
        #         Joy,
        #         1
        #     ]
        # ]

        self.mavlink_msg_available = {}

        for _, topic, msg, queue in self.pub_topics:
            self.mavlink_msg_available[topic] = 0
            self._pub_subscribe_topic(topic, msg, queue)

        # for topic in self.sub_topics:
        #     if len(topic) <= 4:
        #         callback, topic_name, msg, queue = topic
        #         self._sub_subscribe_topic(topic_name, msg, queue, callback)
        #     else:
        #         callback, topic_name, msg, queue, arg = topic
        #         for name in arg:
        #             self._sub_subscribe_topic(topic_name.format(name), msg, queue, callback)

    @staticmethod

    def pub_pass(self):
        pass
    # def _callback_from_topic(topic):
    #     """ Create callback function name

    #     Args:
    #         topic (str): Topic name

    #     Returns:
    #         str: callback name
    #     """
    #     return topic.replace('/', '_') + '_callback'


    def _pub_subscribe_topic(self, topic, msg, queue_size=1):
        """ Subscribe to a topic using the publisher

        Args:
            topic (str): Topic name
            msg (TYPE): ROS message type
            queue_size (int, optional): Queue size
        """
        self.pub.subscribe_topic(self.ROV_name + topic, msg, queue_size)

    # def _sub_subscribe_topic(self, topic, msg, queue_size=1, callback=None):
    #     """ Subscribe to a topic using the subscriber

    #     Args:
    #         topic (str): Topic name
    #         msg (TYPE): ROS message type
    #         queue_size (int, optional): Queue size
    #         callback (None, optional): Callback function
    #     """
    #     self.sub.subscribe_topic(self.ROV_name + topic, msg, queue_size, callback)

    # def _set_servo_callback(self, msg, topic):
    #     """ Set servo from topic

    #     Args:
    #         msg (TYPE): ROS message
    #         topic (TYPE): Topic name

    #     Returns:
    #         None: Description
    #     """
    #     paths = topic.split('/')
    #     servo_id = None
    #     for path in paths:
    #         if 'servo' in path:
    #             servo_id = int(re.search('[0-9]', path).group(0)) + 1
    #             # Found valid id !
    #             break
    #     else:
    #         # No valid id
    #         return

    #     self.set_servo_pwm(servo_id, msg.data)

    # def _set_rc_channel_callback(self, msg, topic):
    #     """ Set RC channel from topic

    #     Args:
    #         msg (TYPE): ROS message
    #         topic (TYPE): Topic name

    #     Returns:
    #         TYPE: Description
    #     """
    #     paths = topic.split('/')
    #     channel_id = None
    #     for path in paths:
    #         if 'rc_channel' in path:
    #             channel_id = int(re.search('[0-9]', path).group(0))  - 1
    #             # Found valid id !
    #             break
    #     else:
    #         # No valid id
    #         return

    #     self.set_rc_channel_pwm(channel_id, msg.data)

    # def _set_mode_callback(self, msg, _):
    #     """ Set ROV mode from topic

    #     Args:
    #         msg (TYPE): Topic message
    #         _ (TYPE): Description
    #     """
    #     self.set_mode(msg.data)

    # def _arm_callback(self, msg, _):
    #     """ Set arm state from topic

    #     Args:
    #         msg (TYPE): ROS message
    #         _ (TYPE): Description
    #     """
    #     self.arm_throttle(msg.data)

    # def _manual_control_callback(self, msg, _):
        
    #     """ Set manual control message from topic
        
    #     Args:
    #         msg (TYPE): ROS message
    #         _ (TYPE): description
    #     """
    #     a=0
        #self.set_manual_control([0,0,0,0], msg.buttons)

    # def _setpoint_velocity_cmd_vel_callback(self, msg, _):
    #     """ Set angular and linear velocity from topic

    #     Args:
    #         msg (TYPE): ROS message
    #         _ (TYPE): Description
    #     """
    #     #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
    #     params = [
    #         None,
    #         None,
    #         None,
    #         msg.twist.linear.x,
    #         msg.twist.linear.y,
    #         msg.twist.linear.z,
    #         None,
    #         None,
    #         None,
    #         None,
    #         None,
    #         ]
    #     self.set_position_target_local_ned(params)

    #     #http://mavlink.org/messages/common#SET_ATTITUDE_TARGET
    #     params = [
    #         None,
    #         None,
    #         None,
    #         None,
    #         msg.twist.angular.x,
    #         msg.twist.angular.y,
    #         msg.twist.angular.z,
    #         None,
    #         ]
    #     self.set_attitude_target(params)
   

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
    
    def _create_header(self, msg):
        """ Create ROS message header

        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    # TODO : tester l'utilisation des vélocités comme dans _create_odometry_msg
    def _create_position_msg(self):
        """ Create odometry message from ROV information

        Raises:
            Exception: No data to create the message
        """
        
        # Check if data is available
        if 'LOCAL_POSITION_NED' not in self.get_data():
            raise Exception('no LOCAL_POSITION_NED data')

        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: Create class to deal with BlueRov state
        msg = PoseStamped()

        self._create_header(msg)

        # http://mavlink.org/messages/common#LOCAL_POSITION_NED
        local_position_data = self.get_data()['LOCAL_POSITION_NED']
        xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
        vxyz_data = [local_position_data[i]  for i in ['vx', 'vy', 'z']]
        msg.pose.position.x = xyz_data[0]
        msg.pose.position.y = xyz_data[1]
        msg.pose.position.z = - xyz_data[2]
        # print(xyz_data)

        # https://mavlink.io/en/messages/common.html#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion

        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)


        # msg.pose.orientation.w = cy * cr * cp + sy * sr * sp
        # msg.pose.orientation.x = cy * sr * cp - sy * cr * sp
        # msg.pose.orientation.y = cy * cr * sp + sy * sr * cp
        # msg.pose.orientation.z = sy * cr * cp - cy * sr * sp
        # on envoie Yaw Pitch Raw à la place du quaternion
        msg.pose.orientation.w = 1
        msg.pose.orientation.x = math.degrees(orientation[0])
        msg.pose.orientation.y = math.degrees(orientation[1])
        msg.pose.orientation.z = math.degrees(orientation[2])
        
        self.pub.set_data('/local_position', msg)



    # def _create_bar30_msg(self):
    #     if 'SCALED_PRESSURE2' not in self.get_data():
    #         raise Exception('no SCALE_PRESSURE2 data')
    #     else :
    #         pass
    #     bar30_data = self.get_data()['SCALED_PRESSURE2']
    #     msg = Bar30()
    #     self._create_header(msg)
    #     msg.time_boot_ms = bar30_data['time_boot_ms']
    #     msg.press_abs    = bar30_data['press_abs']
    #     msg.press_diff   = bar30_data['press_diff']
    #     msg.temperature  = bar30_data['temperature']

    #     self.pub.set_data('/bar30',msg)
    
    # def _create_imu_euler_msg(self):
    #     if 'ATTITUDE' not in self.get_data():
    #         raise Exception('no ATTITUDE data')
    #     else :
    #         pass
    #     #http://mavlink.org/messages/common#ATTITUDE
    #     attitude_data = self.get_data()['ATTITUDE']
    #     orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
    #     orientation_speed = [attitude_data[i] for i in ['rollspeed', 'pitchspeed', 'yawspeed']]
        
    #     msg = Attitude()
    #     self._create_header(msg)
    #     msg.time_boot_ms = attitude_data['time_boot_ms']
    #     msg.roll = orientation[0]
    #     msg.pitch = orientation[1]
    #     msg.yaw = orientation[2]
    #     msg.rollspeed = orientation_speed[0]
    #     msg.pitchspeed = orientation_speed[1]
    #     msg.yawspeed = orientation_speed[2]

    #     self.pub.set_data('/imu/attitude',msg)

    def _create_odometry_msg(self):
        """ Create odometry message from ROV information

        Raises:
            Exception: No data to create the message
        """

        # Check if data is available
        if 'LOCAL_POSITION_NED' not in self.get_data():
            raise Exception('no LOCAL_POSITION_NED data')

        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: Create class to deal with BlueRov state
        msg = Odometry()

        self._create_header(msg)

        #http://mavlink.org/messages/common#LOCAL_POSITION_NED
        local_position_data = self.get_data()['LOCAL_POSITION_NED']
        xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
        vxyz_data = [local_position_data[i]  for i in ['vx', 'vy', 'z']]
        msg.pose.pose.position.x = xyz_data[0]
        msg.pose.pose.position.y = xyz_data[1]
        msg.pose.pose.position.z = xyz_data[2]
        msg.twist.twist.linear.x = vxyz_data[0]/100
        msg.twist.twist.linear.y = vxyz_data[1]/100
        msg.twist.twist.linear.z = vxyz_data[2]/100

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
        orientation_speed = [attitude_data[i] for i in ['rollspeed', 'pitchspeed', 'yawspeed']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)

        msg.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        msg.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
        msg.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
        msg.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp
        msg.twist.twist.angular.x = orientation_speed[0]
        msg.twist.twist.angular.y = orientation_speed[1]
        msg.twist.twist.angular.z = orientation_speed[2]

        self.pub.set_data('/odometry', msg)

    # def _create_imu_msg(self):
    #     """ Create imu message from ROV data

    #     Raises:
    #         Exception: No data available
    #     """

    #     # Check if data is available
    #     if 'ATTITUDE' not in self.get_data():
    #         raise Exception('no ATTITUDE data')

    #     #TODO: move all msgs creating to msg
    #     msg = Imu()

    #     self._create_header(msg)

    #     #http://mavlink.org/messages/common#SCALED_IMU
    #     imu_data = None
    #     for i in ['', '2', '3']:
    #         try:
    #             imu_data = self.get_data()['SCALED_IMU{}'.format(i)]
    #             break
    #         except Exception as e:
    #             pass

    #     if imu_data is None:
    #         raise Exception('no SCALED_IMUX data')

    #     acc_data = [imu_data['{}acc'.format(i)]  for i in ['x', 'y', 'z']]
    #     gyr_data = [imu_data['{}gyro'.format(i)] for i in ['x', 'y', 'z']]
    #     mag_data = [imu_data['{}mag'.format(i)]  for i in ['x', 'y', 'z']]

    #     #http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
    #     msg.linear_acceleration.x = acc_data[0]/100
    #     msg.linear_acceleration.y = acc_data[1]/100
    #     msg.linear_acceleration.z = acc_data[2]/100
    #     msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    #     msg.angular_velocity.x = gyr_data[0]/1000
    #     msg.angular_velocity.y = gyr_data[1]/1000
    #     msg.angular_velocity.z = gyr_data[2]/1000
    #     msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    #     #http://mavlink.org/messages/common#ATTITUDE
    #     attitude_data = self.get_data()['ATTITUDE']
    #     orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]

    #     #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
    #     cy = math.cos(orientation[2] * 0.5)
    #     sy = math.sin(orientation[2] * 0.5)
    #     cr = math.cos(orientation[0] * 0.5)
    #     sr = math.sin(orientation[0] * 0.5)
    #     cp = math.cos(orientation[1] * 0.5)
    #     sp = math.sin(orientation[1] * 0.5)

    #     msg.orientation.w = cy * cr * cp + sy * sr * sp
    #     msg.orientation.x = cy * sr * cp - sy * cr * sp
    #     msg.orientation.y = cy * cr * sp + sy * sr * cp
    #     msg.orientation.z = sy * cr * cp - cy * sr * sp

    #     msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    #     self.pub.set_data('/imu/data', msg)

    # def _create_battery_msg(self):
    #     """ Create battery message from ROV data

    #     Raises:
    #         Exception: No data available
    #     """

    #     # Check if data is available
    #     if 'SYS_STATUS' not in self.get_data():
    #         raise Exception('no SYS_STATUS data')

    #     if 'BATTERY_STATUS' not in self.get_data():
    #         raise Exception('no BATTERY_STATUS data')

    #     bat = BatteryState()
    #     self._create_header(bat)

    #     #http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html
    #     bat.voltage = self.get_data()['SYS_STATUS']['voltage_battery']/1000
    #     bat.current = self.get_data()['SYS_STATUS']['current_battery']/100
    #     bat.percentage = self.get_data()['BATTERY_STATUS']['battery_remaining']/100
    #     self.pub.set_data('/battery', bat)

    def get_battery_percentage(self):
        #récupère simplement le poucentage de la batterie
        bat_percentage = self.get_data()['BATTERY_STATUS']['battery_remaining']/100
        return bat_percentage
    
    def get_current_pose(self) : 
        current_pose = np.array(self.current_pose)
        return current_pose

    # def _create_camera_msg(self):
    #     if not self.video.frame_available():
    #         return
    #     frame = self.video.frame()
    #     image_msg = Image()
    #     self._create_header(image_msg)
    #     height, width, channels = frame.shape
    #     image_msg.width = width
    #     image_msg.height = height
    #     image_msg.encoding = 'bgr8'
    #     image_msg.data = frame
    #     msg = self.video_bridge.cv2_to_imgmsg(frame, "bgr8")
    #     self._create_header(msg)
    #     msg.step = int(msg.step)
    #     self.pub.set_data('/camera/image_raw', msg)
    

    # def _create_ROV_state(self):
    #     """ Create ROV state message from ROV data

    #     Raises:
    #         Exception: No data available
    #     """

    #     # Check if data is available
    #     if 'SERVO_OUTPUT_RAW' not in self.get_data():
    #         raise Exception('no SERVO_OUTPUT_RAW data')

    #     if 'HEARTBEAT' not in self.get_data():
    #         raise Exception('no HEARTBEAT data')

    #     servo_output_raw_msg = self.get_data()['SERVO_OUTPUT_RAW']
    #     servo_output_raw = [servo_output_raw_msg['servo{}_raw'.format(i+1)] for i in range(8)]
    #     motor_throttle = [servo_output_raw[i] - 1500 for i in range(6)]
    #     # 1100 -> -1 and 2000 -> 1
    #     for throttle in motor_throttle:
    #         if throttle < 0:
    #             throttle = throttle/400
    #         else:
    #             throttle = throttle/500

    #     light_on = (servo_output_raw[6] - 1100) / 8
    #     #need to check
    #     camera_angle = servo_output_raw[7] - 1500

    #     # Create angle from pwm
    #     camera_angle = -45*camera_angle/400

    #     base_mode = self.get_data()['HEARTBEAT']['base_mode']
    #     custom_mode = self.get_data()['HEARTBEAT']['custom_mode']

    #     mode, arm = self.decode_mode(base_mode, custom_mode)

    #     data = State()
    #     data.arm = arm
    #     data.rc1 = motor_throttle[0]
    #     data.rc2 = motor_throttle[1]
    #     data.rc3 = motor_throttle[2]
    #     data.rc4 = motor_throttle[3]
    #     data.rc5 = motor_throttle[4]
    #     data.rc6 = motor_throttle[5]
    #     data.light = light_on
    #     data.camera = camera_angle
    #     data.mode = mode
        
    #     self.pub.set_data('/state', data)



    


    def publish(self):
        """ Publish the data in ROS topics
        """
        self.update()
        for sender, topic, _, _ in self.pub_topics:
            try:
                if time.time() - self.mavlink_msg_available[topic] > 1:
                    sender()
            except Exception as e:
                self.mavlink_msg_available[topic] = time.time()
                print(e)





        # position = self.get_data()['LOCAL_POSITION_NED']
        # xyz_data = [position[i]  for i in ['x', 'y', 'z']]
        # x_courant = xyz_data[0]
        # y_courant = xyz_data[1]
        # z_courant = - xyz_data[2]



if __name__ == '__main__':
    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    bluerov = BlueRov(device='udp:localhost:14551')

    # ox = [0.0, 10.0, 10.0, 0.0, 0.0]
    # oy = [0.0, 0.0, 30.0, 30.0, 0.0]
    # oz = [5]
    # resolution = 2

    change_mode = 0

    ox = [-30, -30, -10, -10, -30]
    oy = [-10, 10, 10, -10, -10]
    oz = [-7]
    resolution = 3

    plannification = Plannification()
    goal_evit_1 =  np.array([-19.96 , -10.18])
    goal_evit_2 = np.array([10.24 , 0.02])
    # evitement = Evitement(goal_evit_1)

    goal_scan =  np.array([-9.77 , 0.02])

    x_init_evit_1 = [-19.96, -29.82, -7]
    x_init_evit_2 = [-9.77, 0.02, -7]

    px, py = plannification.planning(ox, oy, resolution)
    px.append(goal_scan[0])
    py.append(goal_scan[1])
    rate = rospy.Rate(50.0)

    yaw_path=150
    yaw_send = False

    # position_desired = [0, 0, 0, 0.5, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2]
    counter_mission = 1
    position_desired = [-100.0, -100.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    while not rospy.is_shutdown():

        bluerov.get_bluerov_data()

        # time.sleep(0.05)
        # evitement.goal_reached = True

        # if evitement.goal_reached == False:
        #     bluerov.do_evit(x_init, goal)

        # if yaw_send == False:
        #     attitude_control.set_target_attitude(bluerov.conn,0, 0, yaw_path)

        # elif bluerov.ok_pose == False:
        #     if change_mode == 0:
        #         bluerov.mission_sent_point = False
                # change_mode += 1

        if counter_mission == 2:
            bluerov.do_evit(x_init_evit_1, goal_evit_1)
            if bluerov.mission_ongoing == False and bluerov.mission_evit == False:
                counter_mission +=1
                print(counter_mission)
        
        elif counter_mission == 1:
            bluerov.do_scan(px, py, oz)
            if bluerov.mission_ongoing == False and bluerov.mission_scan == False:
                counter_mission +=1
                print(counter_mission)

        elif counter_mission == 3:
            bluerov.do_evit(x_init_evit_2, goal_evit_2)
            if bluerov.mission_ongoing == False and bluerov.mission_evit == False:
                counter_mission +=1
                print(counter_mission)

        print("mission_ongoing ", bluerov.mission_ongoing,"| mission_evit ", bluerov.mission_evit, "| mission_scan ", bluerov.mission_scan, "| counter ", counter_mission)
        # yaw_path = (yaw_path+10)%360 

        # bluerov.set_position_target_local_ned(position_desired)
        # counter+=1

        # if counter%10 == 0:
        #     counter = 0
        #     position_desired[0] += 5
        #     position_desired[1] -= 5

#############################TEST MOUVEMENT AVEC VITESSE ET ORIENTATION###########################

        # bluerov.set_speed_and_attitude_target(position_desired)
        # counter+=1

        # if counter%10 == 0:
        #     counter = 0
        #     position_desired[9] += math.pi/4
        #     print(position_desired[9])

#################################################################################################


        # time.sleep(0.05)
        # print(bluerov.get_velodyne_obstacle())
        # print(bluerov.velodyne_data)
        # bluerov.get_collider_obstacles()

        bluerov.publish()

        # bluerov.do_scan([0, 0, 0], [10, 10, 0], 10)
        rate.sleep()