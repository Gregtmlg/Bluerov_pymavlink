#!/bin/bash

cd /media/cellule_ia/Disque/ardupilot/ArduSub

sim_vehicle.py -L Marseille  -S 5  --out=udp:0.0.0.0:14550 --map --console
