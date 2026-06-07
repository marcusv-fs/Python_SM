#!/usr/bin/env python3
import subprocess
import json
import time
import os

ACTOR_NAME = "cubo_2_actor"
COLLISION_MODEL = "cubo_2_collision"
WORLD_NAME = "FRTL_World"  # Deve ser o mesmo nome do seu world

def get_actor_pose():
    """Retorna (x,y,z,roll,pitch,yaw) do ator."""
    # Comando gz topic para obter a última mensagem do tópico de pose do ator
    cmd = f"gz topic -e /model/{ACTOR_NAME}/pose"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        return None
    # A saída é um header + pose protobuf em formato texto, precisamos extrair pos e rot
    lines = result.stdout.strip().split('\n')
    pos = None
    rot = None
    for i, line in enumerate(lines):
        if 'position {' in line:
            pos = {
                'x': float(lines[i+1].split(':')[1].strip()),
                'y': float(lines[i+2].split(':')[1].strip()),
                'z': float(lines[i+3].split(':')[1].strip()),
            }
        elif 'orientation {' in line:
            rot = {
                'x': float(lines[i+1].split(':')[1].strip()),
                'y': float(lines[i+2].split(':')[1].strip()),
                'z': float(lines[i+3].split(':')[1].strip()),
                'w': float(lines[i+4].split(':')[1].strip()),
            }
    if pos is None:
        return None
    # Converter quaternion para Euler? Não precisamos se não houver rotação.
    # Ator não rotaciona, podemos ignorar orientação.
    return pos['x'], pos['y'], pos['z'], 0.0, 0.0, 0.0

def set_collision_pose(x, y, z):
    """Move o modelo de colisão para a posição (x,y,z) usando set_pose."""
    # Monta a requisição JSON para o serviço de set_pose
    request = {
        "entity": {"name": COLLISION_MODEL},
        "pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}
        }
    }
    # Salva em arquivo temporário para passar ao gz service
    with open("/tmp/set_pose_req.json", "w") as f:
        json.dump(request, f)
    cmd = f"gz service -s /world/{WORLD_NAME}/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --reqfile /tmp/set_pose_req.json"
    subprocess.run(cmd, shell=True, capture_output=True)

def main():
    print("Iniciando sincronização da plataforma...")
    last_time = time.time()
    while True:
        pose = get_actor_pose()
        if pose is not None:
            x, y, z, _, _, _ = pose
            set_collision_pose(x, y, z)
            now = time.time()
            if now - last_time > 1.0:  # imprime a cada 1s para evitar flood
                print(f"Sync: ({x:.2f}, {y:.2f}, {z:.2f})")
                last_time = now
        else:
            print("Aguardando pose do ator...")
        time.sleep(0.01)  # ~100 Hz, pode ajustar

if __name__ == "__main__":
    main()