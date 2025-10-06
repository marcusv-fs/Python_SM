#!/bin/bash


echo "Iniciando simulação ArduCopter..."
# gz sim -v4 -r iris_runway.sdf
# 1. Gazebo
gnome-terminal -- bash -c "echo 'Abrindo Gazebo...'; gz sim -v4 -r iris_runway.sdf"

# 2. ArduCopter SITL
gnome-terminal -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --no-mavproxy; exec bash"

# 3. MAVProxy
sleep 4
gnome-terminal -- bash -c "mavproxy.py --master tcp:127.0.0.1:5760 --out 127.0.0.1:14550 --out 127.0.0.1:14551; exec bash"

# 4. QGroundControl
gnome-terminal -- bash -c "~/Área\ de\ trabalho/QGroundControl-x86_64.AppImage; exec bash"

echo "Simulação iniciada!"t