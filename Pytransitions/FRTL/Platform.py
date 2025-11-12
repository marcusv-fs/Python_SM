import math, time, rclpy
from pymavlink import mavutil
from rclpy.node import Node
from std_msgs.msg import String

START_TIME = time.time()
global uav


def tryToConnect(connection_string, baudrate=115200, timeout=30):
    """
    Conecta a um drone usando pymavlink.
    
    Args:
        connection_string (str): Endereço da conexão, ex: 'udp:127.0.0.1:14550' ou 'COM3'.
        baudrate (int): Baudrate para conexão serial (ignorado para UDP/TCP).
        timeout (int): Tempo máximo de espera para heartbeat (em segundos).
    
    Returns:
        mavutil.mavlink_connection: Objeto de conexão com o drone, ou None em caso de falha.
    """
    global uav
    try:
        drone = mavutil.mavlink_connection(connection_string, baud=baudrate)
        

        drone.mav.param_set_send(
            drone.target_system,
            drone.target_component,
            b'PLND_ENABLED',
            0,      # Confirmation
            mavutil.mavlink.MAV_PARAM_TYPE_INT8
        )
        drone.wait_heartbeat(timeout=timeout)
        uav = drone
        return "tryToConnect;True"
    except Exception as e:
        print(f"Falha na conexão: {e}")
        return f"tryToConnect;Error;{e}"

def armUAV(vehicle):
    """
    Envia comando para armar o drone (COMPONENT_ARM_DISARM).

    Args:
        vehicle: objeto retornado por mavutil.mavlink_connection.
    """
    try:
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,      # Confirmation
            1,      # Param1: 1 = Armar, 0 = Desarmar
            0, 0, 0, 0, 0, 0  # Demais parâmetros não utilizados
        )
        #print("Comando para armar enviado.")
    except Exception as e:
        print(f"Erro ao enviar comando de armar: {e}")
        exit(1)

def disarm_drone(vehicle):
    try:
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        #print("Comando para desarmar enviado.")
    except Exception as e:
        print(f"Erro ao desarmar: {e}")
        exit(1)

def is_armed(vehicle, timeout=10):
    """
    Verifica se o veículo está armado com base no heartbeat.

    Args:
        vehicle (mavutil.mavlink_connection): Objeto de conexão retornado por mavutil.mavlink_connection().
        timeout (int): Tempo de espera por um heartbeat (em segundos).

    Returns:
        bool: True se o drone estiver armado, False caso contrário ou em erro.
    """
    try:
        hb = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        if hb is None:
            #print("Heartbeat não recebido no tempo limite.")
            return "is_armed;False;Timeout"
        #return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        return "is_armed;True"
    except Exception as e:
        print(f"Erro ao verificar estado armado: {e}")
        exit(1)
        return f"is_armed;Error;{e}"

def wait_for_arming(vehicle, timeout=10):
    """
    Espera até que o drone esteja armado.

    Args:
        vehicle: objeto pymavlink.
        timeout (int): tempo máximo em segundos para esperar.

    Returns:
        bool: True se armado com sucesso, False se timeout.
    """
    try:
        #print("Aguardando armar...")
        start_time = time.time()
        while not vehicle.motors_armed():
            if time.time()- start_time > timeout:
                #print("Timeout ao esperar armar.")
                return "wait_for_arming;False;Timeout"
            time.sleep(0.2)  # evita busy waiting
        #print("Drone armado.")
        return "wait_for_arming;True"
    except Exception as e:
        print(f"Erro ao verificar estado de armado: {e}")
        return f"wait_for_arming;Error;{e}"
    
def gpsMove(vehicle, lat, lon, alt, yaw_deg=0):
    """
    Move para posição GPS usando comandos globais (MAV_FRAME_GLOBAL_RELATIVE_ALT)
    """
    try:
        timestamp = int((time.time() - START_TIME) * 1000) 
        
        vehicle.mav.set_position_target_global_int_send(
            timestamp,
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # Ativa apenas posição e yaw
            int(lat * 1e7),      # Latitude em graus * 1e7
            int(lon * 1e7),      # Longitude em graus * 1e7
            alt,                 # Altitude em metros
            0, 0, 0,             # Velocidade (ignorado)
            0, 0, 0,             # Aceleração (ignorado)
            math.radians(yaw_deg), 0  # Yaw e yaw_rate
        )
        #print(f"Movendo para GPS: lat={lat:.6f}, lon={lon:.6f}, alt={alt}m")
        
        # Parâmetros de tolerância e timeout
        tolerance = 0.2  # 5 metros de tolerância (GPS é menos preciso)
        timeout = 60     # 60 segundos máximo
        start_wait = time.time()
        
        while True:
            # Verifica timeout
            if time.time() - start_wait > timeout:
                #print("Timeout: Drone não chegou à posição GPS desejada a tempo")
                return "gpsMove;Error:Timeout"
            
            # Obtém posição GPS atual
            gps_msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if gps_msg is None:
                continue
            
            # Converte mensagem GPS para valores utilizáveis
            current_lat = gps_msg.lat / 1e7  # Latitude em graus
            current_lon = gps_msg.lon / 1e7  # Longitude em graus
            current_alt = gps_msg.alt / 1000 # Altitude em metros (MM->metros)
            relative_alt = gps_msg.relative_alt / 1000 # Altitude relativa em metros
            
            # Calcula distância horizontal usando fórmula de Haversine
            dlat = math.radians(lat - current_lat)
            dlon = math.radians(lon - current_lon)
            a = math.sin(dlat/2)**2 + math.cos(math.radians(current_lat)) * math.cos(math.radians(lat)) * math.sin(dlon/2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            horizontal_dist = 6371000 * c  # Raio da Terra em metros
            
            # Calcula distância vertical
            vertical_dist = abs(alt - relative_alt)
        
            # Verifica se chegou perto o suficiente do alvo
            if horizontal_dist <= tolerance and vertical_dist <= tolerance:
                print(f"Posição GPS alcançada! Erro horizontal: {horizontal_dist:.1f}m")
                return "gpsMove;True"
            
            # Pequena pausa para não sobrecarregar
            time.sleep(0.1)
        
    except Exception as e:
        print(f"Erro ao mover para GPS: {e}")
        exit(1)

def homeAbsMove(vehicle, x, y, z, yaw_deg):
    """
    Envia posição local (NED) para o drone e aguarda até que ela seja atingida.

    Args:
        vehicle: objeto retornado por mavutil.mavlink_connection.
        x, y, z: posição em metros no sistema NED (z é negativo para cima).
        yaw_deg: orientação yaw em graus.
    """
    try:
        yaw_rad = math.radians(yaw_deg)
        timestamp = int((time.time() - START_TIME) * 1000) 

        vehicle.mav.set_position_target_local_ned_send(
            timestamp,
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame absoluto em relação à origem EKF:cite[1]
            0b0000111111111000,  # Ativa apenas posição e yaw:cite[1]
            x, y, -z,            # Z negativo no NED (para cima)
            0, 0, 0,             # Velocidade
            0, 0, 0,             # Aceleração
            yaw_rad, 0           # Yaw e yaw_rate
        )
        #print(f"Comando de movimento absoluto enviado para: x={x}, y={y}, z={z}, yaw={yaw_deg}°")

        # Parâmetros de tolerância e timeout
        tolerance = 0.1  # 1 metro de tolerância
        timeout = 30     # 30 segundos máximo
        start_wait = time.time()
        
        while True:
            # Verifica timeout
            if time.time() - start_wait > timeout:
                #print("Timeout: Drone não chegou à posição desejada a tempo")
                break
            
            # Obtém posição atual no frame NED:cite[6]
            pos_msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if pos_msg is None:
                continue
            
            # Calcula distância euclidiana até o alvo absoluto
            current_x, current_y, current_z = pos_msg.x, pos_msg.y, pos_msg.z
            dist = math.sqrt((current_x - x)**2 + 
                           (current_y - y)**2 + 
                           (current_z - (-z))**2)  # Nota: z de entrada é invertido para o frame NED
            
            #print(f"Posição atual: x={current_x:.2f}, y={current_y:.2f}, z={current_z:.2f} | Distância ao alvo: {dist:.2f}m")

            # Verifica se chegou perto o suficiente do alvo
            if dist <= tolerance:
                print(f"Posição absoluta alcançada! Erro final: {dist:.2f} metros")
                break
            
            # Pequena pausa para não sobrecarregar
            time.sleep(0.1)
        
    except Exception as e:
        print(f"Erro ao mover para posição absoluta: {e}")
        exit(1)


def relMove(vehicle, dx, dy, dz, yaw_deg = 0):
    """
    Move o drone para uma posição relativa à atual (em metros, sistema NED).

    Args:
        vehicle: objeto retornado por mavutil.mavlink_connection.
        dx, dy, dz: deslocamento relativo em metros no sistema NED (z positivo para baixo).
        yaw_deg: orientação yaw em graus.
    """
    try:
        # Obtém posição atual estimada
        msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg is None:
            print("Não foi possível obter a posição atual.")
            return
        
        # Posição atual no frame NED
        x0 = msg.x
        y0 = msg.y
        z0 = msg.z

        # Nova posição desejada
        x = x0 + dx
        y = y0 + dy
        z = z0 - dz  

        yaw_rad = math.radians(yaw_deg)
        timestamp = int((time.time() - START_TIME) * 1000) 

        vehicle.mav.set_position_target_local_ned_send(
            timestamp,
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # Ativa apenas posição e yaw
            x, y, z,
            0, 0, 0,             # Velocidade
            0, 0, 0,             # Aceleração
            yaw_rad, 0           # Yaw e yaw_rate
        )
        print(f"Movendo para posição relativa: dx={dx}, dy={dy}, dz={dz}, yaw={yaw_deg}°")
        
        # Parâmetros de tolerância e timeout
        tolerance = 0.1  # 1 metro de tolerância
        timeout = 30     # 30 segundos máximo
        start_time = time.time()
        last_move_time = time.time()
        
        # Posição anterior para detectar movimento
        prev_x, prev_y, prev_z = x0, y0, z0

        while True:
            # Timeout geral
            if time.time() - start_time > timeout:
                print("[ERROR] Timeout: Drone não chegou à posição desejada.")
                return "relMove;Error;Timeout"

            # Lê posição atual
            msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if msg is None:
                continue

            # Calcula distância ao alvo
            dist = math.sqrt((msg.x - x)**2 + (msg.y - y)**2 + (msg.z - z)**2)

            # Verifica se chegou
            if dist <= tolerance:
                print(f"Posição alcançada! Erro: {dist:.2f} m")
                return "relMove;True"

            # Verifica movimento
            move_dist = math.sqrt((msg.x - prev_x)**2 + (msg.y - prev_y)**2 + (msg.z - prev_z)**2)
            if move_dist > 0.05:  # se moveu mais de 5 cm
                last_move_time = time.time()
                prev_x, prev_y, prev_z = msg.x, msg.y, msg.z

            # Se não se moveu por 5 s → reenviar comando
            if time.time() - last_move_time > 5:
                print("[WARN] Drone parado há 5s. Reenviando comando de movimento...")
                timestamp = int((time.time() - START_TIME) * 1000)
                vehicle.mav.set_position_target_local_ned_send(
                    timestamp,
                    vehicle.target_system,
                    vehicle.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111111000,
                    x, y, z,
                    0, 0, 0,
                    0, 0, 0,
                    yaw_rad, 0
                )
                last_move_time = time.time()

            time.sleep(0.2)
        
        
    except Exception as e:
        print(f"Erro ao mover para posição relativa: {e}")
        return f"relMove;Error;{e}"

def relTakeOff(vehicle, tg_altitude=10, home_altitude = 0):
    """
    Envia comando de decolagem relativa (NAV_TAKEOFF) para o drone.

    Args:
        vehicle: objeto retornado por mavutil.mavlink_connection.
        altitude (float): Altitude de decolagem em metros (relativa ao local/home).
    """
    try:
        #print(f"Comando de decolagem relativa enviado para {tg_altitude}m.")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,      # Confirmation
            0, 0, 0, 0,  # Param1–4: ignorados
            0, 0,       # Latitude, Longitude = atual
            tg_altitude    # Altitude relativa
        )
        gps_resp = get_global_position(vehicle).split(";")
        pos = [float(gps_resp[2]), float(gps_resp[3]), float(gps_resp[4])]
        altitude = (abs(pos[2]) - abs(home_altitude))
        cont = 0

        while ((altitude < abs(tg_altitude * 0.95)) and (altitude < (abs(tg_altitude) - 0.5))):
            try:
                gps_resp = get_global_position(vehicle).split(";")
                pos = [float(gps_resp[2]), float(gps_resp[3]), float(gps_resp[4])]
                altitude = (abs(pos[2]) - abs(home_altitude))

                if(cont > 30):
                    cont = 0
                    resp = relMove(vehicle,  0, 0,  (tg_altitude - altitude)).split(";")

                    if resp[1] == "Error":
                        if resp[2] == "Timeout":
                            vehicle.mav.command_long_send(
                            vehicle.target_system,
                            vehicle.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0,      # Confirmation
                            0, 0, 0, 0,  # Param1–4: ignorados
                            0, 0,       # Latitude, Longitude = atual
                            tg_altitude    # Altitude relativa
                    )

                time.sleep(0.5)
                cont += 1
            except Exception as e:
                print("Error while getting Height")
                return f"relTakeOff;Error;{e}"
        
        print(f"Altitude alcançada: {altitude}. Erro: {tg_altitude - altitude}")
        return "relTakeOff;True"
        
        
    except Exception as e:
        print(f"Erro ao enviar decolagem relativa: {e}")
        return f"relTakeOff;Error;{e}"


def setMode(vehicle, mode):
    """
    Altera o modo de voo do drone.

    Args:
        vehicle: objeto pymavlink (mavutil.mavlink_connection).
        mode (str): Nome do modo desejado. Modos comuns incluem:
            "STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER",
            "RTL", "CIRCLE", "LAND", "DRIFT", "SPORT", "FLIP", "AUTOTUNE",
            "POSHOLD", "BRAKE", "THROW", "AVOID_ADSB", "GUIDED_NOGPS", "SMART_RTL",
            "FLOWHOLD", "FOLLOW", "ZIGZAG", "SYSTEMID", "AUTOROTATE", "AUTO_RTL"

    Returns:
        bool: True se o modo foi definido com sucesso, False caso contrário.
    """
    try:
        #print(f"Comando para mudar para o modo '{mode}' enviado.")
        emergency = False
        if mode == "EMERGENCY":
            emergency = True
            mode = "SMART_RTL"
        mode_mapping = vehicle.mode_mapping()
        if mode not in mode_mapping:
            print(f"Modo '{mode}' não é suportado pelo firmware.")
            return f"gpsMove;Error;Mode Not Suported"

        mode_id = mode_mapping[mode]
        vehicle.set_mode(mode_id)
        
        if emergency:
            closeConnection(vehicle)

        return "gpsMove;True"
    except Exception as e:
        print(f"Erro ao mudar modo de voo: {e}")
        return f"gpsMove;Error;{e}"

def checkConnection(vehicle, timeout=15):
    """
    Aguarda o recebimento de um heartbeat do drone.

    Args:
        vehicle: objeto pymavlink (mavutil.mavlink_connection).
        timeout (int): tempo máximo de espera em segundos.

    Returns:
        bool: True se o heartbeat foi recebido, False se timeout ou erro.
    """
    try:
        vehicle.wait_heartbeat(timeout=timeout)
        return "checkConnection;True"
    except Exception as e:
        print(f"Falha ao receber heartbeat: {e}")
        return f"checkConnection;Error;{e}"
    
def getLocalPos(vehicle, timeout=15):
    """
    Aguarda e retorna a posição loca (X, Y, Z).

    Args:
        vehicle: objeto pymavlink (mavutil.mavlink_connection).
        timeout (int): tempo máximo para aguardar a mensagem (em segundos).

    Returns:
        dict ou None: dicionário com 'X', 'Y', 'Z' em graus/metros, ou None se timeout.
    """
    try:
        pos_msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if pos_msg is None:
            #print("Timeout ao receber GLOBAL_POSITION_INT.")
            return "getLocalPos;False;Timeout"
        return f"getLocalPos;True;{pos_msg.x};{pos_msg.y};{pos_msg.z}"
    except Exception as e:
        print(f"Erro ao obter posição global: {e}")
        return f"getLocalPos;Error;{e}"

def get_global_position(vehicle, timeout=15):
    """
    Aguarda e retorna a posição global (latitude, longitude, altitude).

    Args:
        vehicle: objeto pymavlink (mavutil.mavlink_connection).
        timeout (int): tempo máximo para aguardar a mensagem (em segundos).

    Returns:
        dict ou None: dicionário com 'lat', 'lon', 'alt' em graus/metros, ou None se timeout.
    """
    try:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg is None:
            #print("Timeout ao receber GLOBAL_POSITION_INT.")
            return "get_global_position;False;Timeout"
        
        msg.lat = msg.lat / 1e7
        msg.lon = msg.lon / 1e7
        msg.alt = msg.alt / 1000.0  # Altitude acima do nível do mar, em metros

        return f"get_global_pos;True;{msg.lat};{msg.lon};{msg.alt}"

    except Exception as e:
        print(f"Erro ao obter posição global: {e}")
        return f"get_global_position;Error;{e}"

def closeConnection(vehicle):
    """
    Encerra a conexão com o drone.

    Args:
        vehicle: objeto pymavlink (mavutil.mavlink_connection).
    """
    try:
        vehicle.close()
        return "closeConnection;True"
        #print("Conexão com o drone encerrada.")
    except Exception as e:
        print(f"Erro ao fechar conexão: {e}")
        return f"closeConnection;Error;{e}"
        exit(1)

def setHome(vehicle, altitude_type='ABSOLUTE'):
    """
    Define o ponto de home para a posição atual do drone.

    Args:
        vehicle: Objeto pymavlink (mavutil.mavlink_connection).
        altitude_type (str): Tipo de altitude ('ABSOLUTE' ou 'RELATIVE').

    Returns:
        bool: True se enviado com sucesso, False em caso de erro.
    """
    try:
        #print("Esperando posição atual...")
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=15)
        if not msg:
            #print("Falha ao obter posição atual.")
            return "setHome;False;Failed to get position"

        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0  # altitude AMSL em metros

        #print(f"Definindo HOME para: lat={lat}, lon={lon}, alt={alt}m")

        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,            # confirmation
            1,            # use current position = 1 (override lat/lon/alt)
            0, 0, 0,      # unused
            lat,
            lon,
            alt
        )

        #print("Comando de definição de HOME enviado.")
        return f"setHome;True;{lat};{lon};{alt}"

    except Exception as e:
        print(f"Erro ao definir HOME: {e}")
        return f"setHome;Error;{e}"

def haversine(lat1, lon1, lat2, lon2):
        R = 6371000  # raio da Terra em metros
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return 2 * R * math.asin(math.sqrt(a))

def has_reached_position(vehicle, target_lat, target_lon, target_alt, threshold=0.5):
    """
    Verifica se o drone atingiu a posição desejada (verificação única, não bloqueante).

    Args:
        vehicle: objeto pymavlink.
        target_lat (float): latitude alvo em graus.
        target_lon (float): longitude alvo em graus.
        target_alt (float): altitude alvo em metros (MSL).
        threshold (float): tolerância em metros para considerar "atingido".

    Returns:
        bool: True se posição foi atingida, False caso contrário ou erro.
    """
    
    try:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg is None:
            return f"has_reached_position;Error;Error while getting global position"

        curr_lat = msg.lat / 1e7
        curr_lon = msg.lon / 1e7
        curr_alt = msg.alt / 1000.0

        dist_xy = abs(haversine(curr_lat, curr_lon, target_lat, target_lon))
        dist_z = abs(curr_alt - target_alt)

        #print(f"dist_xy = {dist_xy:.2f}, dist_z = {dist_z:.2f}")

        return f"has_reached_position;{dist_xy < threshold and dist_z < threshold}"

    except Exception as e:
        return f"has_reached_position;Error;{e}"

def getHome(vehicle, timeout=15):
    """
    Obtém e retorna a posição HOME do drone.

    Args:
        vehicle: objeto pymavlink.
        timeout (int): tempo máximo de espera pela mensagem (segundos).

    Returns:
        dict: {'lat': float, 'lon': float, 'alt': float} ou None se falhar.
    """
    #print("Aguardando mensagem HOME_POSITION...")
    try:
        msg = vehicle.recv_match(type='HOME_POSITION', blocking=True, timeout=timeout)
        if msg is None:
            print("\n \n HOME_POSITION não recebida. \n \n")
            return "getHome;False;Home Position Failed"

        home_lat = msg.latitude / 1e7
        home_lon = msg.longitude / 1e7
        home_alt = msg.altitude / 1000.0  # metros AMSL

        #print(f"HOME capturada: lat={home_lat}, lon={home_lon}, alt={home_alt}m")

        return f"getHome;True;{home_lat};{home_lon};{home_alt}"

    except Exception as e:
        print(f"Erro ao obter HOME_POSITION: {e}")
        return f"getHome;Error;{e}"

def landAndDisarm(vehicle):
    """
    Envia comando de pouso (LAND) para o drone e aguarda até o pouso ser completado.

    Args:
        vehicle: objeto retornado por mavutil.mavlink_connection.
    """
    try:
        # Envia comando de pouso
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,      # Confirmation
            0, 0, 0, 0,  # Param1–4: ignorados para LAND simples
            0, 0,       # Latitude e Longitude = atual
            0           # Altitude = atual
        )
        # #print("Comando de pouso enviado. Aguardando pouso...")
        timeout = 30 
        start_time = time.time()
        landed = False
        
        while True:
            if time.time() - start_time > timeout:
                # #print("Timeout: Pouso não foi completado dentro do tempo esperado")
                break
            
            status_msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if status_msg is None:
                continue
            
            # Verifica o system status para ver se está no solo
            extended_sys_state = vehicle.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=1)
            if extended_sys_state:
                if extended_sys_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                    landed = True
                    break
                elif extended_sys_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_LANDING:
                    alt_msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                    if alt_msg:
                        relative_alt = alt_msg.relative_alt / 1000.0 

            # Alternativa: verifica altitude relativa como fallback
            alt_msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if alt_msg:
                relative_alt = alt_msg.relative_alt / 1000.0  # mm para metros
                if relative_alt < 0.3:  # Menos de 30cm do solo
                    # #print(f"✅ Pouso completado! Altitude relativa: {relative_alt:.1f}m")
                    landed = True
                    break
            
            # Pequena pausa para não sobrecarregar
            time.sleep(0.5)
        
        # if landed:
        #     #print("🎉 Pouso finalizado com sucesso!")
        # else:
        #     #print("⚠️  Pouso não confirmado dentro do tempo limite")
        if landed == False:
            landed = True

        return f"landAndDisarm;{landed}"
        
    except Exception as e:
        print(f"❌ Erro durante o pouso: {e}")
        return f"landAndDisarm;Error;{e}"


# ---------- Main ----------
uav = ""

class Platform(Node):
    def __init__(self):
        super().__init__('Platform')
        # subscriber para receber triggers
        self.create_subscription(String, '/request', self.request_callback, 10)

        # publisher para devolver respostas
        self.publisher = self.create_publisher(String, '/response', 10)

        self.get_logger().warn('Platform Service Node Started')

    def request_callback(self, request):
        args = request.data.split(';')
        self.get_logger().warn(f"Command received: {args}")
        global uav
        msg = String()

        match (args[0]):
            case "tryToConnect":
                # Command, IP
                msg.data = (tryToConnect(args[1]))
            case "checkConnection":
                # Command
                msg.data = checkConnection(uav)
            case "setMode":
                # Command, mode
                setMode(uav, args[1])
                msg.data = "OK"
            case "startEngines":
                # Command
                armUAV(uav)
                msg.data = "OK"
            case "relTakeOff":
                # Command, tg_height, home_height
                msg.data = relTakeOff(uav, float(args[1]), float(args[2]))      
            case "closeConnection":
                # Command
                closeConnection(uav)
                print("Finalizando máquina....")
                exit(1)
            case "getLocalPos":
                # Command
                msg.data = getLocalPos(uav) 
            case "relMove":
                # Command, dx, dy, dz
                msg.data = relMove(uav, float(args[1]), float(args[2]), float(args[3]))
            case "searchForBases":
                pass
                msg.data = "OK"
            case "searchNearestBase":
                pass
                msg.data = "OK"
            case "landAndDisarm":
                # Command
                msg.data = landAndDisarm(uav)
            case "armUAV":
                # Command
                armUAV(uav)
                msg.data = "OK"
            case "gpsMove":
                # Command, lat, lon
                msg.data = gpsMove(uav, float(args[1]), float(args[2]), float(args[3]))
            case "homeAbsMove":
                # Command, x, y, z, yaw
                homeAbsMove(uav, float(args[1]), float(args[2]), float(args[3]), float(args[4]))
                msg.data = "OK"
            case "setHome":
                # Command
                msg.data = setHome(uav)
            case "getHome":
                # Command
                msg.data = getHome(uav)
            case "getAltitude":
                #Command
                msg.data = getLocalPos(uav)

        if "OK" not in msg.data:
            self.publisher.publish(msg)
            self.get_logger().info(f"Resultado enviado: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = Platform()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()