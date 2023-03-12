# Prérequis

- Ubuntu 20
- python 3.8
- ArduPilot avec ArduSub

# Installer ArduPilot

https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

# Lancer le SITL Bluerov2

Se rendre dans le dossier ArduSub, qui se trouve dans ardupilot, et lancer la commande suivante :

    sim_vehicle.py -L RATBeach --out=udp:0.0.0.0:14550 --map --console

# Packages supplémentaires

Installer le ros-tcp-endpoint et le velodyne-pointcloud comme dans la doc suivante : 

https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux




