from cmath import sqrt
from math import atan2, degrees
import time
from zlib import Z_DEFAULT_COMPRESSION
from numpy import roll
from pymavlink import mavutil
import threading, os
# import getLatLonWL
# import pidRov
import utm
import attitude_control
# import PID



master = mavutil.mavlink_connection('udpin:0.0.0.0:14540') 


X=[0,0,0,0,0,0]

def desideredPosition():
	des_lat=43.6136765412476
	des_lon=13.502154350280763
	
	des_z=-0.6

	'''
	des_x=0
	des_y=0
	des_z=0
	des_roll=0
	des_pitch=0
	des_yaw=0
	'''
	des_utm=utm.from_latlon(des_lat,des_lon)
	des_utm_xy_z=[des_utm[0],des_utm[1],des_z]
	return des_utm_xy_z

def loop_controllo():
	

	#print('ok')
	ok_z=False
	ok_yaw=False
	ok_xy=False
	#yaw_temp=100
	master.set_mode('ALT_HOLD')


	while(1):
		while not attitude_control.is_armed(master):
			master.arducopter_arm()
        
	#leggo i miei dati
		x_rov=X[0]
		y_rov=X[1]
		z_rov=X[2]
		roll_rov=X[3]
		pitch_rov=X[4]
		yaw_rov=degrees(X[5])
		if yaw_rov<0:
			yaw_ok=yaw_rov+360
		else:
			yaw_ok=yaw_rov


	
#		diff_x=u_rovpos[0]-z_utm_des[0]
#		diff_y=u_rovpos[1]-z_utm_des[1]
#		dist = sqrt((diff_x)^2+(diff_y)^2)

		#yaw_path=degrees(atan2(diff_y,diff_x))
		yaw_path=330

		z_utm_des=desideredPosition()		
		
		diff_z= z_utm_des[2] + z_rov
		#print(z_utm_des[2],diff_z,z_rov)
		if abs(diff_z) <= 0.2:
			ok_z=True
			print('ok depth')
		else:
			ok_z=False
			attitude_control.set_target_depth(z_utm_des[2],master)
		
		#print(z_rov,yaw_rov)
		if ok_z == True and ok_yaw==False:
			attitude_control.set_target_attitude(master,0, 0, yaw_path)
		
		diff_yaw= yaw_path - yaw_ok
			
		if abs(diff_yaw) <= 5:
			ok_yaw=True
			print('yaw_ok')
		else:
			ok_yaw=False

		time.sleep(0.01)
		
		if ok_z== True and ok_yaw==True and ok_xy==False:
			print('avanza')
			#u_x=pidRov.main(dist)
			master.mav.manual_control_send( #x - y- z - r - button 
    		master.target_system, #x,y and r will be between [-1000 and 1000].
    		500, # x, da sostituire con un pid . 
    		0, #y
  		    500, #la Z va da -1000 a 1000, quindi 500 è ferma
    		0, #r è lo yaw
    		0)
		else:
			master.mav.manual_control_send( #x - y- z - r - button 
    		master.target_system, #x,y and r will be between [-1000 and 1000].
    		0, # x, da sostituire con un pid . 
    		0, #y
  		    500, #la Z va da -1000 a 1000, quindi 500 è ferma
    		0, #r è lo yaw
    		0)



loop_controllo()