#!/usr/bin/env python3

import math
import time
import os
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity}] {message}"

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.get_logger().info('Nó DroneController iniciado.')

        self.master = None
        self.target_system = 0
        self.target_component = 0
        self.heartbeat_thread = None
        self.heartbeat_stop_event = threading.Event()
        
        # Variáveis de estado agora pertencem à classe
        self.start_time = time.time()
        self.home_pos = [0.0, 0.0, 0.0]  # Latitude, Longitude, Altitude

        self.subscription = self.create_subscription(
            String, '/request', self.request_callback, 10)
        self.publisher = self.create_publisher(String, '/response', 10)

    def _send_mavlink_command(self, command_id, confirmation, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        if not self.master:
            self.get_logger().error("Conexão MAVLink não estabelecida.")
            return False
        try:
            self.master.mav.command_long_send(
                self.target_system, self.target_component,
                command_id, confirmation,
                param1, param2, param3, param4, param5, param6, param7
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar comando MAVLink {command_id}: {e}")
            return False

    def _wait_for_message(self, msg_type, timeout=5):
        if not self.master:
            return None
        return self.master.recv_match(type=msg_type, blocking=True, timeout=timeout)

    def _heartbeat_monitor(self):
        while not self.heartbeat_stop_event.is_set():
            if self.master:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    mavutil.mavlink.MAV_MODE_GUIDED_ARMED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
            time.sleep(1)

    def connect_drone(self, connection_string, baudrate=115200, timeout=30):
        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
            self.master.wait_heartbeat(timeout=timeout)
            self.target_system = self.master.target_system
            self.target_component = self.master.target_component

            self.heartbeat_stop_event.clear()
            self.heartbeat_thread = threading.Thread(target=self._heartbeat_monitor, daemon=True)
            self.heartbeat_thread.start()
            
            return "tryToConnect;True"
        except Exception as e:
            self.master = None
            return f"tryToConnect;Error;{e}"

    def arm_uav(self):
        if self._send_mavlink_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1):
            return "armUAV;True"
        return "armUAV;False"

    def disarm_uav(self):
        if self._send_mavlink_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0):
            return "disarmUAV;True"
        return "disarmUAV;False"

    def is_armed(self, timeout=5):
        hb = self._wait_for_message('HEARTBEAT', timeout=timeout)
        if hb:
            armed_state = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            return f"is_armed;{armed_state}"
        return "is_armed;False;Timeout"

    def set_mode(self, mode_name):
        if not self.master:
            return "setMode;Error;NoConnection"
        
        mode_name = mode_name.upper()
        if mode_name == "EMERGENCY":
            mode_name = "SMART_RTL"
            
        mode_mapping = self.master.mode_mapping()
        if mode_name not in mode_mapping:
            return f"setMode;Error;ModeNotSupported"

        self.master.set_mode(mode_mapping[mode_name])
        return f"setMode;True;{mode_name}"

    def check_connection(self, timeout=15):
        try:
            self.master.wait_heartbeat(timeout=timeout)
            return "checkConnection;True"
        except Exception as e:
            return f"checkConnection;Error;{e}"

    def get_local_position(self, timeout=5):
        pos_msg = self._wait_for_message('LOCAL_POSITION_NED', timeout=timeout)
        att_msg = self._wait_for_message('ATTITUDE', timeout=timeout)
        if pos_msg and att_msg:
            return f"getLocalPos;True;{pos_msg.x};{pos_msg.y};{pos_msg.z};{att_msg.yaw}"
        return "getLocalPos;False;Timeout"

    def get_global_position(self, timeout=5):
        msg = self._wait_for_message('GLOBAL_POSITION_INT', timeout=timeout)
        if msg:
            return f"getGlobalPos;True;{msg.lat / 1e7};{msg.lon / 1e7};{msg.alt / 1000.0}"
        return "getGlobalPos;False;Timeout"

    def set_home(self, timeout=15):
        pos_resp = self.get_global_position(timeout=timeout).split(';')
        if pos_resp[1] == 'True':
            lat, lon, alt = float(pos_resp[2]), float(pos_resp[3]), float(pos_resp[4])
            if self._send_mavlink_command(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 1, 0, 0, 0, lat, lon, alt):
                self.home_pos = [lat, lon, alt]
                return f"setHome;True;{lat};{lon};{alt}"
        return "setHome;False;Failed"

    def get_home(self, timeout=15):
        msg = self._wait_for_message('HOME_POSITION', timeout=timeout)
        if msg:
            return f"getHome;True;{msg.latitude / 1e7};{msg.longitude / 1e7};{msg.altitude / 1000.0}"
        return "getHome;False;Timeout"

    def absolute_move(self, x, y, z, yaw_deg=None, tolerance=0.1, timeout=30):
        """
        Move o drone para uma coordenada absoluta no referencial LOCAL_NED,
        mantendo a orientação (Yaw) fixa para não rodar a câmara.
        """
        if not self.master: return "AbsMove;Error;NoConnection"

        # 1. Obter a posição e orientação atual para bloquear o Yaw
        local_data = self.get_local_position().split(';')
        if local_data[1] != 'True': return "AbsMove;Error;CannotGetLocalPosition"
        
        current_yaw_rad = float(local_data[5])

        # Se não for passado um Yaw específico, usamos o atual (Yaw Lock)
        if yaw_deg is None:
            target_yaw_rad = current_yaw_rad
        else:
            target_yaw_rad = math.radians(yaw_deg)

        start_wait = time.time()

        # MÁSCARA: 0b0000101111111000 
        # (Bit 10 em 0 indica que o drone deve OBEDECER ao target_yaw_rad)
        type_mask = 0b0000101111111000

        self.get_logger().info(f"A mover para Absoluto: X={x}, Y={y}, Z={z} | Yaw={math.degrees(target_yaw_rad):.1f}º")

        while time.time() - start_wait < timeout:
            timestamp = int((time.time() - self.start_time) * 1000)
            
            # Enviamos o comando repetidamente para garantir que a controladora mantém o setpoint
            self.master.mav.set_position_target_local_ned_send(
                timestamp, self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
                x, y, z,                # Posição alvo
                0, 0, 0,                # Velocidade (ignorada)
                0, 0, 0,                # Aceleração (ignorada)
                target_yaw_rad, 0       # Yaw alvo e Yaw Rate (ignorado)
            )

            # 2. Verificar progresso usando telemetria em tempo real (sem sleep extra)
            pos_msg = self._wait_for_message('LOCAL_POSITION_NED', timeout=0.5)
            if pos_msg:
                # Cálculo da distância 3D até ao alvo
                dist = math.sqrt((x - pos_msg.x)**2 + (y - pos_msg.y)**2 + (z - pos_msg.z)**2)
                
                if dist <= tolerance:
                    self.get_logger().info("Alvo absoluto atingido.")
                    return "AbsMove;True"

        self.get_logger().error("Timeout no movimento absoluto.")
        return "AbsMove;Error;Timeout"

    def relative_move(self, dx, dy, dz, yaw_deg=None, tolerance=0.1, timeout=30):
        if not self.master: return "relMove;Error;NoConnection"

        local_pos = self.get_local_position().split(';')
        if local_pos[1] != 'True': return "relMove;Error;CannotGetLocalPosition"

        x = float(local_pos[2]) + dx
        y = float(local_pos[3]) + dy
        z = float(local_pos[4]) - dz
        
        # O get_local_position já retorna o yaw em radianos no índice 5
        current_yaw_rad = float(local_pos[5])
        
        # Se não passarmos um Yaw específico, travamos no Yaw atual
        if yaw_deg is None:
            yaw_rad = current_yaw_rad
        else:
            yaw_rad = math.radians(yaw_deg)

        start_wait = time.time()

        # MÁSCARA NOVA: 0b0000101111111000 (O bit 10 está zerado, então o drone vai respeitar o Yaw enviado)
        type_mask = 0b0000101111111000

        while time.time() - start_wait < timeout:
            timestamp = int((time.time() - self.start_time) * 1000)
            self.master.mav.set_position_target_local_ned_send(
                timestamp, self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
                x, y, z, 0, 0, 0, 0, 0, 0, yaw_rad, 0
            )

            # Usar o método _wait_for_message ao invés de time.sleep para evitar o buffer lag
            pos_msg = self._wait_for_message('LOCAL_POSITION_NED', timeout=1)
            if pos_msg:
                dist = math.dist([x, y, z], [pos_msg.x, pos_msg.y, pos_msg.z])
                if dist <= tolerance: 
                    return "relMove;True"
                    
        return "relMove;Error;Timeout"

    def relative_takeoff(self, target_altitude=10.0, tolerance=0.5, timeout=120):
        if not self.master: return "relTakeOff;Error;NoConnection"

        count = 0
        
        self._send_mavlink_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_altitude)
        
        start_time = time.time()
        last_alt = 0
        stuck_count = 0

        self.get_logger().info(f"Decolagem iniciada para {target_altitude}m...")

        while time.time() - start_time < timeout:
            # Usamos LOCAL_POSITION_NED pois ela nos dá a posição E a velocidade vertical (vz)
            msg = self._wait_for_message('LOCAL_POSITION_NED', timeout=1)
            
            if msg:
                # No sistema NED, o eixo Z é positivo para BAIXO. 
                # Portanto, altitude = -z
                current_alt = -msg.z
                vertical_velocity = msg.vz # Velocidade vertical (positiva para baixo)
                
                self.get_logger().info(f"Alt: {current_alt:.2f}m | Vz: {vertical_velocity:.2f}")

                # Condição de Sucesso: Chegou na faixa de tolerância
                if abs(current_alt - target_altitude) <= tolerance:
                    self.get_logger().info("Decolagem concluída com sucesso.")
                    return "relTakeOff;True"

                # Lógica de Correção (Cenário B):
                # Se a velocidade vertical for muito baixa (drone parou de subir) 
                # e ele ainda está abaixo do alvo:
                if abs(vertical_velocity) < 0.1 and current_alt < (target_altitude - tolerance):
                    stuck_count += 1
                    if stuck_count > 40: # Se ficar "preso" por ~3 segundos
                        self.get_logger().warn("Drone estagnado abaixo do alvo. Enviando correção de subida...")
                        
                        # Forçamos o drone a ir para a altitude exata usando comando de posição
                        # (O mesmo que você usou no absolute_move)
                        type_mask = 0b0000111111111000 # Máscara padrão para posição
                        self.master.mav.set_position_target_local_ned_send(
                            int((time.time() - self.start_time) * 1000),
                            self.target_system, self.target_component,
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            type_mask,
                            msg.x, msg.y, -target_altitude, # Mantém X e Y, ajusta Z
                            0, 0, 0, 0, 0, 0, 0, 0
                        )
                        stuck_count = 0 # Reseta o contador após corrigir
            
        return "relTakeOff;Error;Timeout"

    def land(self, timeout=120):
        if not self.master: return "land;Error;NoConnection"

        self._send_mavlink_command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        start_time = time.time()

        while time.time() - start_time < timeout:
            extended_sys_state = self._wait_for_message('EXTENDED_SYS_STATE', timeout=1)
            if extended_sys_state and extended_sys_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                return "land;True"
            time.sleep(1)
        return "land;False;Timeout"

    def back_home(self):
        if self.home_pos == [0.0, 0.0, 0.0]: return "backHome;Error;HomeNotSet"
        if "True" in self.absolute_move(self.home_pos[0], self.home_pos[1], self.home_pos[2] + 4):
            return self.land()
        return "backHome;False;Failed"

    def close_connection(self):
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_stop_event.set()
        if self.master:
            self.master.close()
            self.master = None
            return "closeConnection;True"
        return "closeConnection;False;NoActiveConnection"

    def _haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        dphi, dlambda = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
        a = math.sin(dphi / 2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlambda / 2)**2
        return 2 * R * math.asin(math.sqrt(a))

    def request_callback(self, request):
        # Dispara uma thread imediatamente para não bloquear o ROS2
        threading.Thread(target=self._process_command, args=(request.data,), daemon=True).start()

    def _process_command(self, data):
        args = data.split(';')
        command = args[0]
        result = ""
        self.get_logger().info(f"Received: {";".join(args)}")

        try:
            if command == "tryToConnect": result = self.connect_drone(args[1])
            elif command == "checkConnection": result = self.check_connection()
            elif command == "setMode": result = self.set_mode(args[1])
            elif command in ["startEngines", "armUAV"]: result = self.arm_uav()
            elif command == "relTakeOff": result = self.relative_takeoff(float(args[1]))
            elif command == "getLocalPos": result = self.get_local_position()
            elif command == "relMove": result = self.relative_move(float(args[1]), float(args[2]), float(args[3]))
            elif command == "land": result = self.land()
            elif command == "AbsMove": result = self.absolute_move(float(args[1]), float(args[2]), float(args[3]))
            elif command == "homeAbsMove": result = self.relative_move(float(args[1]), float(args[2]), float(args[3]), float(args[4]))
            elif command == "setHome": result = self.set_home()
            elif command == "getHome": result = self.get_home()
            elif command == "getAltitude": result = self.get_local_position()
            elif command == "backHome": result = self.back_home()
            elif command == "closeConnection":
                result = self.close_connection()
                rclpy.shutdown()
                return
            else: result = "ERROR: Unknown Command"
        except Exception as e:
            result = f"ERROR: Exception: {e}"
            self.get_logger().error(f"Received: {result}")
            

        response_msg = String()
        response_msg.data = result
        self.get_logger().warn(f"Sending: {result}")
        self.publisher.publish(response_msg)

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller.close_connection()
        drone_controller.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()