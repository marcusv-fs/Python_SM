#!/bin/bash

echo "Iniciando simulação ArduCopter..."

# 1. Gazebo
gnome-terminal -- bash -i -c "
  export GZ_SIM_RESOURCE_PATH=\$HOME/gz_ws/src/ardupilot_gazebo/models:\$HOME/gz_ws/src/ardupilot_gazebo/worlds:\${GZ_SIM_RESOURCE_PATH};
  gz sim -v4 -r FRTL/FRTL_World.sdf; exit
"

# 2. ArduCopter SITL
gnome-terminal -- bash -i -c "
  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --no-mavproxy; exit
"

# 3. MAVProxy
sleep 4
gnome-terminal -- bash -i -c "
  mavproxy.py --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550 --out 127.0.0.1:14551; exit
"

# 4. QGroundControl
gnome-terminal -- bash -i -c "
  ~/Área\ de\ trabalho/QGroundControl-x86_64.AppImage; exit
"

echo "Simulação iniciada!"
