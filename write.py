from pymavlink import mavutil


# fonction pour envoyer une position à rejoindre au robot
def set_position_target_local_ned(the_connection, param=[]):
    """ Create a SET_POSITION_TARGET_LOCAL_NED message
        http://mavlink.org/messages/common#SET_POSITION_TARGET_LOCAL_NED

    Args:
        param (list, optional): param1, param2, ..., param11
    """
    if len(param) != 11:
        print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')

    # Set mask
    # mask = 0b0000000111111111
    # for i, value in enumerate(param):
    #     if value is not None:
    #         mask -= 1<<i
    #     else:
    #         param[i] = 0.0

    #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
    the_connection.mav.set_position_target_local_ned_send(
        0,                                              # system time in milliseconds
        the_connection.target_system,                        # target system
        the_connection.target_component,                     # target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,            # frame
        0b110111111000,                                           # mask
        param[0], param[1], param[2],                   # position x,y,z
        param[3], param[4], param[5],                   # velocity x,y,z
        param[6], param[7], param[8],                   # accel x,y,z
        param[9], param[10])                            # yaw, yaw rate


# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

while 1:
    set_position_target_local_ned(the_connection, [5,5,-5, 0, 0, 0, 0, 0, 0, 0, 0])
