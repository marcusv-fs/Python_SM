import math
import time
from pymavlink import mavutil  # type: ignore

uav[]

drone[drone]

def connect_drone(connection_string, baudrate=115200, timeout=30):
    """
    Conecta a um drone usando pymavlink.
    
    Args:
        connection_string (str): Endereço da conexão, ex: 'udp:127.0.0.1:14550' ou 'COM3'.
        baudrate (int): Baudrate para conexão serial (ignorado para UDP/TCP).
        timeout (int): Tempo máximo de espera para heartbeat (em segundos).
    
    Returns:
        mavutil.mavlink_connection: Objeto de conexão com o drone, ou None em caso de falha.
    """
    try:
        drone = mavutil.mavlink_connection(connection_string, baud=baudrate)
        #print("Conectando... aguardando heartbeat.")
        drone.wait_heartbeat(timeout=timeout)
        #print(f"Conectado ao sistema (ID={drone.target_system}, Componente={drone.target_component})")
        return drone
    except Exception as e:
        #print(f"Falha na conexão: {e}")
        exit(1)
        return None

def update_value(configFile, key):
    try:
        with open(configFile, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if '=' in line:
                    current_key, valor = line.split('=', 1)
                    if current_key.strip() == key:
                        return valor.strip()
        return None  # Retorna None se a chave não for encontrada
    except FileNotFoundError:
        #print("Value not found!")
        exit(1)
        return None

def arm_drone(vehicle):
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
        #print(f"Erro ao enviar comando de armar: {e}")
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
        #print(f"Erro ao desarmar: {e}")
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
            return False
        return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    except Exception as e:
        #print(f"Erro ao verificar estado armado: {e}")
        exit(1)
        return False

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
                return False
            time.sleep(0.2)  # evita busy waiting
        #print("Drone armado.")
        return True
    except Exception as e:
        #print(f"Erro ao verificar estado de armado: {e}")
        exit(1)
        return False
    
def move_to_gps_absolute(vehicle, start_time, lat, lon, alt, yaw_deg=0):
    """
    Move para posição GPS usando comandos globais (MAV_FRAME_GLOBAL_RELATIVE_ALT)
    """
    try:
        timestamp = int((time.time() - start_time) * 1000)
        
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
        print(f"Movendo para GPS: lat={lat:.6f}, lon={lon:.6f}, alt={alt}m")
        
        # Parâmetros de tolerância e timeout
        tolerance = 0.2  # 5 metros de tolerância (GPS é menos preciso)
        timeout = 60     # 60 segundos máximo
        start_wait = time.time()
        
        while True:
            # Verifica timeout
            if time.time() - start_wait > timeout:
                #print("Timeout: Drone não chegou à posição GPS desejada a tempo")
                break
            
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
            
            # Distância 3D total (priorizando horizontal)
            total_dist = math.sqrt(horizontal_dist**2 + vertical_dist**2)
            
            #print(f"Distância ao alvo: Horizontal={horizontal_dist:.1f}m, Vertical={vertical_dist:.1f}m, Total={total_dist:.1f}m")
            
            # Verifica se chegou perto o suficiente do alvo
            if horizontal_dist <= tolerance and vertical_dist <= tolerance:
                print(f"Posição GPS alcançada! Erro horizontal: {horizontal_dist:.1f}m")
                break
            
            # Pequena pausa para não sobrecarregar
            time.sleep(0.1)
        
    except Exception as e:
        print(f"Erro ao mover para GPS: {e}")

def move_to_absolute(vehicle, start_time, x, y, z, yaw_deg):
    """
    Envia posição local (NED) para o drone e aguarda até que ela seja atingida.

    Args:
        vehicle: objeto retornado por mavutil.mavlink_connection.
        start_time: tempo inicial (ex: time.time()no início da missão).
        x, y, z: posição em metros no sistema NED (z é negativo para cima).
        yaw_deg: orientação yaw em graus.
    """
    try:
        yaw_rad = math.radians(yaw_deg)
        timestamp = int((time.time()- start_time) * 1000)

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
        #print(f"Erro ao mover para posição absoluta: {e}")
        exit(1)

def move_to_relative(vehicle, start_time, dx, dy, dz, yaw_deg):
    """
    Move o drone para uma posição relativa à atual (em metros, sistema NED).

    Args:
        vehicle: objeto retornado por mavutil.mavlink_connection.
        start_time: tempo inicial (ex: time.time()).
        dx, dy, dz: deslocamento relativo em metros no sistema NED (z positivo para baixo).
        yaw_deg: orientação yaw em graus.
    """
    try:
        # Obtém posição atual estimada
        msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg is None:
            #print("Não foi possível obter a posição atual.")
            return
        
        # Posição atual no frame NED
        x0 = msg.x
        y0 = msg.y
        z0 = msg.z

        #print("Pos atual NED: x: ", x0, " , y: ", y0, " z: ", z0)

        # Nova posição desejada
        x = x0 + dx
        y = y0 + dy
        z = z0 - dz  

        #print("Pos desejada NED: x: ", x, " , y: ", y, " z: ", z)

        yaw_rad = math.radians(yaw_deg)
        timestamp = int((time.time() - start_time) * 1000)

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
        timeout = 10     # 30 segundos máximo
        start_wait = time.time()
        
        while True:
            # Verifica timeout
            if time.time() - start_wait > timeout:
                #print("Timeout: Drone não chegou à posição desejada a tempo")
                break
            
            # Obtém posição atual
            pos_msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
            if pos_msg is None:
                continue
            
            # Calcula distância até o alvo
            dist = math.sqrt((pos_msg.x - x)**2 + 
                           (pos_msg.y - y)**2 + 
                           (pos_msg.z - z)**2)
            
            # Verifica se chegou perto o suficiente do alvo
            if dist <= tolerance:
                print(f"Posição alcançada! Erro: {dist:.2f} metros")
                break
            
            # Pequena pausa para não sobrecarregar
            time.sleep(0.1)
        
    except Exception as e:
        #print(f"Erro ao mover para posição relativa: {e}")
        exit(1)

def takeoff_relative(vehicle, tg_altitude=10, home_altitude = 0):
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
        pos = get_global_position(vehicle)
        cont = 0
        altitude = (abs(pos['alt']) - abs(home_altitude))
        while ((altitude < abs(tg_altitude * 0.95)) and (altitude < (abs(tg_altitude) - 0.5))):
            try:
                pos = get_global_position(vehicle)
                altitude = (abs(pos['alt']) - abs(home_altitude))
                #print(f"Altitude: {altitude} - tg_Altitude: {tg_altitude}")

                if(cont > 20):
                    print("ERRORRRRR")
                    cont = 0
                    move_to_relative(vehicle, time.time(), 0, 0, tg_altitude + 0.1, 0)
                    cont = 0

                time.sleep(0.2)
                cont += 1
            except:
                #print("Error while getting Height")
                exit(1)   
        
        print(f"Altitude alcançada: {altitude}. Erro: {tg_altitude - altitude}")
        
        
    except Exception as e:
        #print(f"Erro ao enviar decolagem relativa: {e}")
        exit(1)

def set_mode(vehicle, mode):
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
            mode = "RTL"
        mode_mapping = vehicle.mode_mapping()
        if mode not in mode_mapping:
            #print(f"Modo '{mode}' não é suportado pelo firmware.")
            return False

        mode_id = mode_mapping[mode]
        vehicle.set_mode(mode_id)
        
        if emergency:
            close_connection(vehicle)

        return True
    except Exception as e:
        #print(f"Erro ao mudar modo de voo: {e}")
        exit(1)
        return False

def wait_for_heartbeat(vehicle, timeout=5):
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
        return 1
    except Exception as e:
        #print(f"Falha ao receber heartbeat: {e}")
        exit(1)
        return 2
    
def get_local_position(vehicle, timeout=5):
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
            return None
        return pos_msg.x, pos_msg.y, pos_msg.z
    except Exception as e:
        #print(f"Erro ao obter posição global: {e}")
        exit(1)
        return None

def get_global_position(vehicle, timeout=5):
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
            return None

        # Conversão de coordenadas: lat/lon em 1E7, alt em mm
        return {
            'lat': msg.lat / 1e7,
            'lon': msg.lon / 1e7,
            'alt': msg.alt / 1000.0  # Altitude acima do nível do mar, em metros
        }
    except Exception as e:
        #print(f"Erro ao obter posição global: {e}")
        exit(1)
        return None

def close_connection(vehicle):
    """
    Encerra a conexão com o drone.

    Args:
        vehicle: objeto pymavlink (mavutil.mavlink_connection).
    """
    try:
        vehicle.close()
        #print("Conexão com o drone encerrada.")
    except Exception as e:
        #print(f"Erro ao fechar conexão: {e}")
        exit(1)

def set_home_to_current_position(vehicle, altitude_type='ABSOLUTE'):
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
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if not msg:
            #print("Falha ao obter posição atual.")
            return False

        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0  # altitude AMSL em metros

        #print(f"Definindo HOME para: lat={lat}, lon={lon}, alt={alt}m")

        # altitude_type: 0 = absolute, 1 = relative
        alt_type_flag = 0 if altitude_type == 'ABSOLUTE' else 1

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
        return True

    except Exception as e:
        #print(f"Erro ao definir HOME: {e}")
        exit(1)
        return False

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
            return False

        curr_lat = msg.lat / 1e7
        curr_lon = msg.lon / 1e7
        curr_alt = msg.alt / 1000.0

        dist_xy = abs(haversine(curr_lat, curr_lon, target_lat, target_lon))
        dist_z = abs(curr_alt - target_alt)

        #print(f"dist_xy = {dist_xy:.2f}, dist_z = {dist_z:.2f}")

        return dist_xy < threshold and dist_z < threshold

    except:
        return False


def get_home_position(vehicle, timeout=5):
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
            #print("HOME_POSITION não recebida.")
            return None

        home_lat = msg.latitude / 1e7
        home_lon = msg.longitude / 1e7
        home_alt = msg.altitude / 1000.0  # metros AMSL

        #print(f"HOME capturada: lat={home_lat}, lon={home_lon}, alt={home_alt}m")

        return {'lat': home_lat, 'lon': home_lon, 'alt': home_alt}

    except Exception as e:
        #print(f"Erro ao obter HOME_POSITION: {e}")
        exit(1)
        return None

def land_and_disarm(vehicle):
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
        timeout = 50 
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
                # MAV_LANDED_STATE: 0=undef, 1=on ground, 2=in air, 3=takeoff, 4=landing
                if extended_sys_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                    # #print("✅ouso completado! Drone está no solo.")
                    landed = True
                    break
                elif extended_sys_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_LANDING:
                    alt_msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
                    if alt_msg:
                        relative_alt = alt_msg.relative_alt / 1000.0  # mm para metros
                        # #print(f"Pousando... Altitude relativa: {relative_alt:.1f}m")
            
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
            
        return landed
        
    except Exception as e:
        #print(f"❌ Erro durante o pouso: {e}")
        return False
