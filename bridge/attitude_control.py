# Import mavutil
from cmath import pi
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math
import time
import sys
ALT_HOLD_MODE = 2

def is_armed(master):
    try:
        return bool(master.wait_heartbeat().base_mode & 0b10000000)
    except:
        return False

def mode_is(mode):
    try:
        return bool(master.wait_heartbeat().custom_mode == mode)
    except:
        return False

def set_target_depth(depth,master):
    master.mav.set_position_target_global_int_send(
        0,     
        0, 0,   
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT, # frame
        0b0000111111111000,
        0,0, depth,
        0 , 0 , 0 , # x , y , z velocity in m/ s ( not used )
        0 , 0 , 0 , # x , y , z acceleration ( not supported yet , ignored in GCS Mavlink )
        0 , 0 ) # yaw , yawrate ( not supported yet , ignored in GCS Mavlink )

def set_target_attitude(master,roll, pitch, yaw, control_yaw=True):
    bitmask = (1<<6 | 1<<3)  if control_yaw else 1<<6

    master.mav.set_attitude_target_send(
        0,     
        0, 0,   
        bitmask,
        QuaternionBase([math.radians(roll), math.radians(pitch), math.radians(yaw)]), # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        0, #roll rate
        0, #pitch rate
        0, 0)    # yaw rate, thrust 

# Create the connection
#master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
#funziona se qground è chiuso
'''
master = mavutil.mavlink_connection('udpin:0.0.0.0:14440') 
#funziona solo se qground è aperto
# impostare questa porta su inoltro mavlink

# Wait a heartbeat before sending commands
print(master.wait_heartbeat())


while not is_armed():
    master.arducopter_arm()

while not mode_is(ALT_HOLD_MODE):
    master.set_mode('ALT_HOLD')



def get_data(): #Loop comunicazione con QGROUND
	
	# Get some information !
	while True:
                
		try:
			msg= master.recv() #LEGGO i messaggi di QGROUDN
			#print(len(msg))
			if len(msg)>0:
				campi=master.mav.parse_char(msg)

				
				if campi.get_type()=="AHRS2":
					#print ("roll:", campi.roll, "pitch:", campi.pitch, "yaw:", campi.yaw)
					Xg=[campi.roll,campi.pitch,campi.yaw]
					print(Xg)
					return(Xg)
	
								
					
		except:
			pass



ok_z=False
zd=-1
while(1):

    X=get_data()
    yaw_temp= X[2]
    yaw_z = float(yaw_temp)
    yy = (yaw_z*180)/pi
    if yy <0:
        yaw_ok=yy+360
    else:
        yaw_ok=yy
    print(yaw_ok)
    time.sleep(1)

#set_target_depth(-0.5)
#time.sleep(2)


# commentiamo il test dello yaw 

pitch = yaw = roll = 0
#for i in range(500): 
yaw = 250
#set_target_attitude(roll, pitch, yaw) 

'''
