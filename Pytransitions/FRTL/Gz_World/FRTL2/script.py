import math

x_amp = 1.5
y_amp = 3.0
z = 0.5
duration = 40.0
step = duration / 1000  # 0.04 s

with open("/home/marcusv-fs/Programacao/Python_SM/Pytransitions/FRTL/Gz_World/FRTL2/waypoints.xml", "w") as f:
    for i in range(1001):  # 1001 pontos (0 a 40s inclusive)
        t_sec = i * step
        t_rad = t_sec * (2 * math.pi / duration)
        x = x_amp * math.sin(t_rad)
        y = y_amp * math.sin(2 * t_rad)
        f.write(f'<waypoint>{x:.3f} {y:.3f} {z}</waypoint>\n')