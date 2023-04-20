import rospy
from bluerov_node import BlueRov
import dynamic_window_approach as dywa
import numpy as np

import math
from enum import Enum

import matplotlib.pyplot as plt

try:
    rospy.init_node('user_node', log_level=rospy.DEBUG)
except rospy.ROSInterruptException as error:
    print('pubs error with ROS: ', error)
    exit(1)
bluerov = BlueRov(device='udp:localhost:14551')

change_mode = 0

ox = [1.866, 10.44, 1.9, -7.12, 1.866]
oy = [-21.355, -26.35, -41.22, -35.59, -21.355]
oz = [-7.409]
resolution = 1

# plannification = Plannification()
goal =  np.array([-47.96 , -11.19])
evitement = dywa.Evitement(goal)

x_init = [-37.77, -11.42, -7.409]

#px, py = plannification.planning(ox, oy, resolution)
rate = rospy.Rate(50.0)

yaw_path=150
yaw_send = False

def main(gx=goal[0],  gy=goal[1], robot_type=dywa.RobotType.rectangle):
    print(__file__ + " start!!")

    while not rospy.is_shutdown():

        bluerov.get_bluerov_data()

        if evitement.goal_reached == False and bluerov.init_evit == False:
            bluerov.do_evit(evitement, x_init, goal)
            
        if evitement.goal_reached == False and bluerov.init_evit == True:
            x, u, predicted_trajectory = bluerov.do_evit(evitement, x_init, goal)
            trajectory = np.vstack((trajectory, x))

            if dywa.show_animation:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
                plt.plot(x[0], x[1], "xr")
                plt.plot(goal[0], goal[1], "xb")
                if bluerov.ob.size != 0:
                    plt.plot(bluerov.ob[:, 0], bluerov.ob[:, 1], "ok")
                dywa.plot_robot(x[0], x[1], x[2], dywa.config)
                dywa.plot_arrow(x[0], x[1], x[2])
                plt.axis("equal")
                plt.grid(True)
                plt.pause(0.0001)

            # check reaching goal
            dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
            if dist_to_goal <= dywa.config.robot_radius:
                print("Goal!!")
                break

        bluerov.publish()

    print("Done")
    if dywa.show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()





    rate.sleep()

if __name__ == '__main__':
    main(robot_type=dywa.RobotType.rectangle)
    # main(robot_type=RobotType.circle)