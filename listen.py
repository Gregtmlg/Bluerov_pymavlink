from pymavlink import mavutil

# Start a connection listening on a UDP port (14540 or 14550)
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages

while 1:
    msg_position = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking = True)
# ATTENTION : axe des z vers le bas pour LOCAL_POSITION_NED
    print("local_position_ned")

    msg_quaternion = the_connection.recv_match(type='ATTITUDE_QUATERNION', blocking = True)

    print("attitude_quaternion")
# # Reconstruction similaire message geometry ros PoseStamped
    msg = dict({'Pose' : dict({'x' : msg_position.x, 'y' : msg_position.y, 'z' : -msg_position.z}), 
               'Orientation' : dict({'x' : msg_quaternion.q2, 'y' : msg_quaternion.q3, 'z' : msg_quaternion.q4, 'w' : msg_quaternion.q1})})
    

    print(msg_position)