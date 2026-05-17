import os, time, rclpy, threading, math

import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity}] {message}"

from transitions.extensions import GraphMachine
from rclpy.node import Node
from std_msgs.msg import UInt8, String
from dataclasses import dataclass
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from DetectNet import detect_single_frame

DSD_LAND_ALT = -0.5
SMOOTH = 0.4 
TARGET_HEIGHT = 4

resp = ['']
msg = String()
global Wait_flag
Wait_flag = False

@dataclass
class Position:
    X: float
    Y: float
    Z: float
    YAW: float

@dataclass
class Target:
    id: int
    pos: Position
    visited: bool = False

def is_base_already_visited(self, x, y):
        """Verifica se a base detectada está próxima de uma já visitada"""
        for v_x, v_y in self.visited_bases_coords:
            dist = math.sqrt((x - v_x)**2 + (y - v_y)**2)
            if dist < self.min_dist_to_new_base:
                return True
        return False

import math

def isSafeToLand(self):
        # Pousa se X/Y forem perto E se estiver baixo (< 0.6m)
        return (calcDist(self.basePos) < self.SAFE_DISTANCE) and (self.basePos.Z <= 0.6)

def optimizeBasePositions(self):
        self.node.get_logger().info("Iniciando limpeza das bases mapeadas...")
        while len(self.bases) > 5:
            min_dist = float('inf')
            pair_to_merge = None

            # 1. Encontra o par de bases com a menor distância entre si
            for i in range(len(self.bases)):
                for j in range(i + 1, len(self.bases)):
                    b1 = self.bases[i]
                    b2 = self.bases[j]
                    dist = math.sqrt((b1.pos.X - b2.pos.X)**2 + (b1.pos.Y - b2.pos.Y)**2)
                    
                    if dist < min_dist:
                        min_dist = dist
                        pair_to_merge = (i, j)
            
            # 2. Mescla o par mais próximo
            if pair_to_merge:
                i, j = pair_to_merge
                b1 = self.bases[i]
                b2 = self.bases[j]
                
                # Calcula o centroide (média das coordenadas)
                avg_x = (b1.pos.X + b2.pos.X) / 2.0
                avg_y = (b1.pos.Y + b2.pos.Y) / 2.0
                avg_z = (b1.pos.Z + b2.pos.Z) / 2.0
                
                merged_target = Target(id=b1.id, pos=Position(avg_x, avg_y, avg_z, 0), visited=False)
                self.bases.pop(j)
                self.bases.pop(i)
                
                # Adiciona a nova base
                self.bases.append(merged_target)
                self.node.get_logger().info(f"Bases fundidas para corrigir odometria. Distância de erro era: {min_dist:.2f}m")

        # 3. Reorganiza os IDs
        for idx, base in enumerate(self.bases):
            base.id = idx + 1
            self.node.get_logger().info(f"Small Base Final #{base.id} -> X: {base.pos.X:.2f}, Y: {base.pos.Y:.2f}")

        if len(self.bases) < 5:
            self.node.get_logger().warn(f"Atenção: Apenas {len(self.bases)} bases foram encontradas!")

def searchBases(self, img):
    """
    Cria e adiciona novos Targets à lista self.bases com base nas 
    inferências da rede neural.
    """

    time.sleep(3)

    self.dronePos = updateDronePos(self)
    if not self.dronePos:
        return self.bases

    # 1. Solicita a detecção
    self.node.get_logger().info("Executando detect_single_frame...")
    annotated_img, detections = detect_single_frame(img)

    # 2. Publica a imagem anotada
    if annotated_img is None:
        self.node.get_logger().error("annotated_img é None! DetectNet não retornou imagem.")
        return self.bases  

    self.node.get_logger().info(f"Publicando imagem anotada, shape: {annotated_img.shape}")
    out_msg = self.node.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
    self.node.annotated_pub.publish(out_msg)
    
    # Constante óptica validada pelo SDF
    FOCAL_LENGTH = 405.47 
    
    current_yaw = getattr(self, 'droneYaw', 0.0)

    # 3. Processa as detecções
    for det in detections:
        if isinstance(det, dict) and 'force_vector' in det and 'width' in det:
            
            label = det.get('label', 'small_base') # assume small_base se não houver label
            fx = det['force_vector']['x']
            fy = det['force_vector']['y']
            bbox_width_pixels = det['width']

            if bbox_width_pixels <= 0:
                continue

            if label == 'large_base':
                real_size = 2.0
            else:
                real_size = 1.1

            # A) Calcula a distância linear usando o tamanho correto
            dist_to_base = (real_size * FOCAL_LENGTH) / bbox_width_pixels

            # B) Coordenadas no referencial do DRONE (Body Frame)
            dx_body = -(fy * dist_to_base)
            dy_body = (fx * dist_to_base)

            # C) TRANSFORMAÇÃO PARA REFERENCIAL GLOBAL (NED)
            dx_ned = dx_body * math.cos(current_yaw) - dy_body * math.sin(current_yaw)
            dy_ned = dx_body * math.sin(current_yaw) + dy_body * math.cos(current_yaw)
            
            est_x = self.dronePos.X + dx_ned
            est_y = self.dronePos.Y + dy_ned
            est_z = self.dronePos.Z 

            is_duplicate = False
            for target in self.bases:
                dist = math.sqrt((est_x - target.pos.X)**2 + (est_y - target.pos.Y)**2)
                if dist < 1.0:
                    is_duplicate = True
                    break

            # 5. Adiciona novo Target respeitando a regra de posições
            if not is_duplicate:
                if label == 'large_base':
                    self.largeBase = Target(id=99, pos=Position(est_x, est_y, est_z, 0), visited=False)
                    self.node.get_logger().info(f"Large Base encontrada em: X={est_x:.2f}, Y={est_y:.2f}")
                else:
                    new_id = len(self.bases) + 1
                    new_target = Target(id=new_id, pos=Position(est_x, est_y, est_z, 0), visited=False)
                    self.bases.append(new_target)
                    self.node.get_logger().info(f"Nova Small Base #{new_id} em: X={est_x:.2f}, Y={est_y:.2f}")

    return self.bases

def searchNearestBase(self, img):
    """
    Calcula o deslocamento relativo necessário MANTENDO NO REFERENCIAL BODY.
    """
    time.sleep(3)

    self.dronePos = updateDronePos(self)
    if not self.dronePos:
        return Position(0.0, 0.0, 0.0, 0.0)

    current_yaw = getattr(self, 'droneYaw', 0.0) 
    
    self.node.get_logger().info("Executando detect_single_frame...")
    annotated_img, detections = detect_single_frame(img)

    if annotated_img is None:
        self.node.get_logger().error("annotated_img é None!")
        return Position(0.0, 0.0, 0.0, 0.0)

    out_msg = self.node.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
    self.node.annotated_pub.publish(out_msg)
    
    if not detections:
        return Position(0.0, 0.0, 0.0, 0.0)

    best_rel_pos = None
    min_dist_sq = float('inf')

    smooth_factor = 0.85 
    deadzone = 0.05      

    FOCAL_LENGTH = 205.47
    
    # Constantes óticas (Pixels / FOCAL_LENGTH)
    # Assumindo resolução de 640x480 e force_vector normalizado de -1 a 1
    CONST_X = (640.0 / 2.0) / FOCAL_LENGTH  # Aproximadamente 1.557
    CONST_Y = (480.0 / 2.0) / FOCAL_LENGTH  # Aproximadamente 1.168

    for det in detections:
        if isinstance(det, dict) and 'force_vector' in det and 'width' in det:
            
            # 1. CORREÇÃO DE ESCALA (Tamanho Dinâmico)
            label = det.get('label', 'small_base')
            if label == 'large_base':
                real_size = 2.0
            else:
                real_size = 1.1

            fx = det['force_vector']['x']
            fy = det['force_vector']['y']
            bbox_width_pixels = det['width']

            dist_to_base = (real_size * FOCAL_LENGTH) / bbox_width_pixels

            if abs(fx) < deadzone: fx = 0.0
            if abs(fy) < deadzone: fy = 0.0

            # 2. CORREÇÃO MATEMÁTICA PINHOLE (Aplicando CONST_X e CONST_Y)
            dx_body = -(fy * CONST_Y * dist_to_base) * smooth_factor
            dy_body = (fx * CONST_X * dist_to_base) * smooth_factor

            # 3. Transformação para Global APENAS para verificar se já foi visitada!
            dx_ned = dx_body * math.cos(current_yaw) - dy_body * math.sin(current_yaw)
            dy_ned = dx_body * math.sin(current_yaw) + dy_body * math.cos(current_yaw)

            est_x_global = self.dronePos.X + dx_ned
            est_y_global = self.dronePos.Y + dy_ned

            if is_base_already_visited(self, est_x_global, est_y_global):
                continue

            # 4. CORREÇÃO DE REFERENCIAL 
            dist_sq = dx_body**2 + dy_body**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                # Salva dx_body, dy_body e A DISTÂNCIA Z ATÉ A BASE!
                best_rel_pos = Position(dx_body, dy_body, dist_to_base, 0.0)

    return best_rel_pos if best_rel_pos is not None else Position(0.0, 0.0, 0.0, 0.0)

def updateDronePos(self):
    try:
        Request(self.node, f"getLocalPos")
        Wait()
        return self.dronePos

    except Exception as e:
        print(f"Erro ao atualizar a posição do drone: {e}")
        Request(self.node, f"setMode;EMERGENCY")
        rclpy.shutdown()

def calcDist(basePos: Position):
    return math.sqrt(
    (basePos.X) ** 2 +
    (basePos.Y) ** 2
)

def markVisitedBases(self, current_position: Position):
    closestBaseID = 0
    dist = 9999999.9
    for base in self.bases:
        auxDist = math.sqrt((base.pos.X - current_position.X) ** 2 +(base.pos.Y - current_position.Y) ** 2)
        if dist > auxDist:
            dist = auxDist
            closestBaseID = base.id
        
    for base in self.bases:
        if closestBaseID == base.id:
            base.visited = True

def setHome(self):
        Request(self.node, f"setHome")
        Wait()
        return self.homePos

def tryToConnect(self):
    Request(self.node, f"tryToConnect;{self.connection_string}")
    Wait()
    return self.isConnected

def Wait():
    start_time = time.time()
    global Wait_flag
    Wait_flag = False
    while Wait_flag == False:
        time.sleep(1)
        now = time.time()
        if now - start_time > 60:  # Timeout de 30 segundos
            print("Timeout ao esperar resposta.")
            valor = input("Digite 'Y' para prosseguir")
            Wait_flag = True
    Wait_flag = False

def Request(node: Node, command : str):
    msg.data = command
    node.request_pub.publish(msg)

class FrtlNode(Node):
    def __init__(self):
        super().__init__('machine1_node')
        self.get_logger().info("Node iniciado, criando máquina de estados...")
        self.machine = FRTL(self)
        self.bridge = CvBridge()
        self.frame = None
        # subscriber para receber triggers
        self.create_subscription(UInt8, '/trigger_start', self.start_callback, 10)
        # subscriber para receber comandos
        self.create_subscription(String, '/response', self.response_callback, 10)
        # subscriber para receber imagens da câmera
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # publisher para requisitar ações da plataforma
        self.request_pub = self.create_publisher(String, '/request', 10)
        # publisher view do Drone
        self.annotated_pub = self.create_publisher(Image, '/ProcDroneImg', 10)

    def start_callback(self, msg: UInt8):
        self.machine.trigger_start = True
        self.machine.phase = msg.data
        self.get_logger().info(f"\n Command received: {self.machine.trigger_start}, in: {time.time()} ###")
        if(msg.data == 2):
            msg = String()
            msg.data = f"setMode;EMERGENCY"
            self.request_pub.publish(msg)
            rclpy.shutdown()

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        try:
            self.machine.mission.img = self.frame
            self.machine.mission.trigger_image = True
        except AttributeError:
            pass

    def response_callback(self, msg: String):
        global Wait_flag
        resp = msg.data.split(';')
        self.get_logger().warn(str(resp))
        error = False

        match resp[0]:
            case "getLocalPos":
                if resp[1] == "True":
                    self.machine.mission.dronePos.X = float(resp[2])
                    self.machine.mission.dronePos.Y = float(resp[3])
                    self.machine.mission.dronePos.Z = float(resp[4])
                    self.machine.mission.dronePos.YAW = float(resp[5])
                    Wait_flag = True
                else:
                    error = True
            case "setHome":
                if resp[1] == "True":
                    self.machine.homePos.X = float(resp[2])
                    self.machine.homePos.Y = float(resp[3])
                    self.machine.homePos.Z = float(resp[4])
                    Wait_flag = True
                else:
                    error = True
            case "tryToConnect":
                if resp[1] == "True":
                    self.machine.isConnected = True
                    Wait_flag = True
                else:
                    error = True

            case "relTakeOff" | "relMove" | "land" | "AbsMove" | "setMode" | "armUAV" | "backHome" | "closeConnection":
                if resp[1] == "True":
                    Wait_flag = True
                else:
                    error = True

        if error:
            self.get_logger().error("; ".join(resp))

class Phase1(GraphMachine):
    ####################### States Declaration #######################   
    states = ['Initial', 'Explore', 'SearchForBases', 'GoToBase', 'ApproachToBase', 'LandAndScore', 'TakeOff', 'Final']

    ####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_Explore', 'source': 'Initial', 'dest': 'Explore', 'before': 'before_Initial_Explore'},

        {'trigger': 'Explore_to_SearchForBases', 'source': 'Explore', 'dest': 'SearchForBases', 'conditions': 'cond_Explore_SearchForBases', 'before': 'reset_trigger_image'},

        {'trigger': 'SearchForBases_to_Explore', 'source': 'SearchForBases', 'dest': 'Explore', 'conditions': 'cond_SearchForBases_Explore', 'before': 'before_SearchForBases_Explore'},
        {'trigger': 'SearchForBases_to_GoToBase', 'source': 'SearchForBases', 'dest': 'GoToBase', 'conditions': 'cond_SearchForBases_GoToBase', 'before': 'before_SearchForBases_GoToBase'},

        {'trigger': 'GoToBase_to_ApproachToBase', 'source': 'GoToBase', 'dest': 'ApproachToBase'},

        {'trigger': 'ApproachToBase_to_ApproachToBase', 'source': 'ApproachToBase', 'dest': 'ApproachToBase', 'conditions': 'cond_ApproachToBase_ApproachToBase', 'before': 'before_ApproachToBase_ApproachToBase'},
        {'trigger': 'ApproachToBase_to_LandAndScore', 'source': 'ApproachToBase', 'dest': 'LandAndScore', 'conditions': 'cond_ApproachToBase_LandAndScore'},

        {'trigger': 'LandAndScore_to_TakeOff', 'source': 'LandAndScore', 'dest': 'TakeOff'},

        {'trigger': 'TakeOff_to_GoToBase', 'source': 'TakeOff', 'dest': 'GoToBase', 'conditions': 'cond_TakeOff_GoToBase'},
        {'trigger': 'TakeOff_to_Final', 'source': 'TakeOff', 'dest': 'Final', 'conditions': 'cond_TakeOff_Final'},
    ]

    def __init__(self, node: Node, homePos: Position, TARGET_HEIGHT: float):    
        super().__init__(
            model=self,
            name="Phase1",
            states=self.states,
            transitions=self.transitions,
            initial='Initial',
            auto_transitions=False,
            show_conditions=True,
            show_state_attributes=True
        )
        
        self.visited_bases_coords = [] # Lista para salvar posições (X, Y) já visitadas
        self.min_dist_to_new_base = 1.0 # Distância mínima para considerar uma base como "nova"

        self.targetHeight = TARGET_HEIGHT
        self.homePos = homePos
        self.finished = False
        self.node = node
        self.start_time = time.time()


        self.detections = []
        self.img = None
        self.trigger_image = False
        self.count = 0
        self.bases: list[Target] = []
        self.largeBase = None  # Para guardar a base final separada
        self.visitedBases = 0  
        
        # Grid Lawnmower para cobrir 8x8 (pontos extremos e meios)
        self.searchPoses = [ 
            Position( 0 + self.homePos.X,   0 + self.homePos.Y, self.homePos.Z - self.targetHeight, 0), 
            Position( 0 + self.homePos.X,  -8.5 + self.homePos.Y, self.homePos.Z - self.targetHeight, 0),
            Position( -4 + self.homePos.X,  -4 + self.homePos.Y, self.homePos.Z - self.targetHeight, 0),
            Position(-8 + self.homePos.X,  -0 + self.homePos.Y, self.homePos.Z - self.targetHeight, 0),
            Position(-8 + self.homePos.X,   -8 + self.homePos.Y, self.homePos.Z - self.targetHeight, 0)
        ]
        self.qtdPos = len(self.searchPoses)
        self.basePos = Position(0,0,0, 0)
        self.dronePos = Position(0,0,0, 0)
        self.distToTarget = 99999999.0

        self.SAFE_DISTANCE = 0.2

        ####################### Draw State Machine ######################
        try:
            out_dir = 'Pytransitions/FRTL/Data/'
            os.makedirs(out_dir, exist_ok=True)
            
            self.get_graph().draw(os.path.join(out_dir, 'Phase1.canon'), prog='dot')
            self.get_graph().draw(os.path.join(out_dir, 'Phase1.png'), prog='dot')

        except Exception as e:
            self.node.get_logger().error(f"Não foi possível gerar diagrama: {e}")

####################### Transition Conditions ####################### 
    def cond_Explore_SearchForBases(self):
        return self.count <= self.qtdPos and self.trigger_image

    def cond_SearchForBases_Explore(self):
        return self.count < self.qtdPos
            
    def cond_SearchForBases_GoToBase(self):
        return self.count >= self.qtdPos
    
    def cond_ApproachToBase_ApproachToBase(self):
        return not isSafeToLand(self)
    
    def cond_ApproachToBase_LandAndScore(self):
        return isSafeToLand(self)
    
    def cond_TakeOff_GoToBase(self):
        return self.visitedBases < len(self.bases) 
    
    def cond_TakeOff_Final(self):
        return self.visitedBases >= len(self.bases)
    
    ####################### Before Transitions ####################### 
    def before_Initial_Explore(self):
        self.count = 0
        self.visitedBases = 0

    def reset_trigger_image(self):
        self.trigger_image = False

    def before_SearchForBases_Explore(self):
        Request(self.node, f"homeAbsMove;{self.searchPoses[self.count].X};{self.searchPoses[self.count].Y};{self.searchPoses[self.count].Z}; 0")
        Wait()
        self.count = self.count + 1

    def before_SearchForBases_GoToBase(self):
        optimizeBasePositions(self)

    def before_ApproachToBase_ApproachToBase(self):
        print("Moving to new Position...")
        delta_z = (DSD_LAND_ALT - self.dronePos.Z) * SMOOTH
        Request(self.node, f"relMove;{self.basePos.X};{self.basePos.Y};{-delta_z}")
        Wait()

    ####################### On_enter States #######################         

    def on_enter_SearchForBases(self):
        self.node.get_logger().info("on_enter_SearchForBases")
        while self.img is None:
            self.node.get_logger().info("Aguardando imagem...")
            time.sleep(0.1)
        self.node.get_logger().info("Chamando searchBases...")
        self.bases = searchBases(self, self.img)
        self.node.get_logger().info(f"Bases encontradas: {len(self.bases)}")

    def on_enter_GoToBase(self):
        self.node.get_logger().info("on_enter_GoToBase")

        if self.visitedBases < len(self.bases):
            target_global = self.bases[self.visitedBases].pos
            self.node.get_logger().info(f"Indo para Small Base {self.visitedBases+1}")
        else:
            self.node.get_logger().error("Erro: Sem mais bases para visitar no GoToBase!")
            return
            
        self.dronePos = updateDronePos(self)
        current_yaw = getattr(self, 'droneYaw', 0.0)
    
        delta_x_global = target_global.X - self.dronePos.X
        delta_y_global = target_global.Y - self.dronePos.Y
        
        delta_x_body = delta_x_global * math.cos(current_yaw) + delta_y_global * math.sin(current_yaw)
        delta_y_body = -delta_x_global * math.sin(current_yaw) + delta_y_global * math.cos(current_yaw)
        delta_z = 0.1

        self.node.get_logger().info(f"Movendo (Body Frame): Frente={delta_x_body:.2f}m, Direita={delta_y_body:.2f}m")

        # 3. Envia o comando relativo na perspectiva do drone
        Request(self.node, f"relMove;{delta_x_body};{delta_y_body};{delta_z}")
        Wait()

    def on_enter_ApproachToBase(self):
        self.node.get_logger().info("on_enter_ApproachToBase")

        while self.trigger_image == False:
            time.sleep(0.1)
        self.trigger_image = False

        self.basePos = searchNearestBase(self, self.img)     
        self.distToTarget = calcDist(basePos=self.basePos)

    def on_enter_LandAndScore(self):
        """Callback executado ao iniciar o pouso"""
        self.node.get_logger().info("Pousando e salvando posição...")
        self.dronePos = updateDronePos(self)
        self.visited_bases_coords.append((self.dronePos.X, self.dronePos.Y))
        Request(self.node, "land")
        Wait()
        self.visitedBases += 1

    def on_enter_TakeOff(self):
        self.node.get_logger().info("on_enter_TakeOff")
        print("Changing to Guided...")
        Request(self.node, f"setMode;GUIDED")
        Wait()

        print("Arming...")
        Request(self.node, f"armUAV")
        Wait()

        print("TakingOff...")
        Request(self.node, f"relTakeOff;{self.targetHeight};{self.homePos.Z}")
        Wait()

    def on_enter_Final(self):
        self.node.get_logger().info("on_enter_Phase1_Final")
        self.finished = True

    ####################### run #######################   
    def run(self):
        while not self.finished:
            self.tock = 1
            for transition in self.transitions:
                if self.state in str(transition.get("source")):
                    if self.may_trigger(transition.get("trigger")):
                        self.tock = 0
                        self.node.get_logger().info(f"Transition triggered: {transition.get("trigger")}")
                        self.trigger(transition.get("trigger"))
            if self.tock == 1:
                print("tock -> ")
                time.sleep(0.5) 
        
class FRTL(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial', 'Connect', 'Wait', 'StartEngines', 'TakeOff', 'Phases', 'Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_Connect', 'source': 'Initial', 'dest': 'Connect'},
        {'trigger': 'Connect_to_Connect', 'source': 'Connect', 'dest': 'Connect', 'conditions': 'cond_Connect_Connect', 'before': ['before_Connect_Connect']},
        {'trigger': 'Connect_to_Wait', 'source': 'Connect', 'dest': 'Wait', 'conditions': 'cond_Connect_Wait'},
        {'trigger': 'Connect_to_Final', 'source': 'Connect', 'dest': 'Final', 'conditions': 'cond_Connect_Final'},
        {'trigger': 'Wait_to_StartEngines', 'source': 'Wait', 'dest': 'StartEngines', 'conditions': 'cond_Wait_StartEngines', 'before': ['reset_trigger_start',]},
        {'trigger': 'StartEngines_to_TakeOff', 'source': 'StartEngines', 'dest': 'TakeOff'},
        {'trigger': 'TakeOff_to_Phases', 'source': 'TakeOff', 'dest': 'Phases'},
        {'trigger': 'Phases_to_Final', 'source': 'Phases', 'dest': 'Final', 'before': 'before_Phases_Final'}
        ]
    
    def __init__(self, node: Node):    
        super().__init__(
            model=self,
            name="FRTL",
            states=self.states,
            transitions=self.transitions,
            initial='Initial',
            auto_transitions=False,
            show_conditions=True,
            show_state_attributes=True
        )

        self.connection_string = "udp:127.0.0.1:14551"
        self.finished = False
        self.node = node
        self.start_time = time.time()

        self.isConnected = False 
        self.phase = 0
        self.connectionTrys = 0
        self.homePos = Position(0,0,0, 0)
        self.targetHeight = TARGET_HEIGHT
        self.mission = None

        self.trigger_start = False

        ####################### Draw State Machine ######################
        try:
            out_dir = 'Pytransitions/FRTL/Data'
            os.makedirs(out_dir, exist_ok=True)
            self.get_graph().draw(os.path.join(out_dir, 'FRTL.canon'), prog='dot')
            self.get_graph().draw(os.path.join(out_dir, 'FRTL.png'), prog='dot')

        except Exception as e:
            self.node.get_logger().error(f"Não foi possível gerar diagrama: {e}")

####################### Transition Conditions ####################### 
    def cond_Connect_Connect(self) -> bool:
        return ((not self.isConnected) and self.connectionTrys < 3)
    
    def cond_Connect_Wait(self):
        return self.isConnected
    
    def cond_Connect_Final(self):
        return not (self.isConnected and self.connectionTrys >= 3)
    
    def cond_Wait_StartEngines(self):
        return self.trigger_start

####################### Before Transitions ####################### 
    def reset_trigger_start(self):
        self.trigger_start = False

    def before_Connect_Connect(self):
        self.connectionTrys = self.connectionTrys + 1

    def before_Phases_Final(self):
        print("Returning to Launch")
        Request(self.node, f"backHome")
        Wait()

####################### On_enter States #######################         
    def on_enter_Connect(self):
        self.node.get_logger().info("on_enter_Connect")
        self.isConnected = tryToConnect(self)
        self.connectionTrys = self.connectionTrys + 1

    def on_enter_Wait(self):
        self.node.get_logger().info("on_enter_Wait")
        print("Waiting for start...\n")
        self.homePos = setHome(self)
        

    def on_enter_StartEngines(self):
        self.node.get_logger().info("on_enter_StartEngines")
        Request(self.node, f"setMode;GUIDED")
        Wait()
        Request(self.node, f"armUAV")
        Wait()

    def on_enter_TakeOff(self):
        self.node.get_logger().info("on_enter_TakeOff")
        Request(self.node, f"relTakeOff;{self.targetHeight};{self.homePos.Z}")
        Wait()        

    def on_enter_Phases(self):
        self.node.get_logger().info("on_enter_Phases")
        if self.phase == 1:
            self.mission = Phase1(self.node, self.homePos, TARGET_HEIGHT)
            self.mission.run()

    def on_enter_Final(self):
        self.node.get_logger().info("on_enter_Final")
        Request(self.node, f"closeConnection")
        Wait()
        self.finished = True

####################### run #######################   
    def run(self):
        while not self.finished:
            self.tock = 1
            for transition in self.transitions:
                if self.state in str(transition.get("source")):
                    if self.may_trigger(transition.get("trigger")):
                        self.tock = 0
                        self.trigger(transition.get("trigger"))
            
            print(self.state)
            
            if self.tock == 1:
                print("tock -> ")
                time.sleep(0.5) 
            else:
                self.node.get_logger().info("Not Tock")

######################## MAIN ########################
def ros_spin_thread(node):
    """Thread dedicada ao ROS2 spin."""
    rclpy.spin(node)

def machine_thread(machine):
    """Thread dedicada à máquina de estados."""
    machine.run()
    return 

def main(args=None):
    rclpy.init(args=args)
    node = FrtlNode()

    # Cria threads
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    fsm_thread = threading.Thread(target=machine_thread, args=(node.machine,), daemon=True)

    # Inicia as threads
    ros_thread.start()
    fsm_thread.start()
    try:
        fsm_thread.join()
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")
    finally:
        node.get_logger().info("Encerrando ROS 2...")
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2.0) 
        print("Programa encerrado!")

if __name__ == '__main__':
    main()
