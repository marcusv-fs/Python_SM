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

TARGET_HEIGHT = 3
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

def searchBases(self, img):
    """
    Cria e adiciona novos Targets à lista self.bases com base nas 
    inferências da rede neural, evitando duplicatas em um raio de 1m.
    """

    self.dronePos = updateDronePos(self)

    # 1. Solicita a detecção (Certifique-se de que detect_single_frame foi importado)
    self.node.get_logger().info("Executando detect_single_frame...")
    annotated_img, detections = detect_single_frame(img)

    # 2. Publica a imagem usando o nó ROS (self.node)
    if annotated_img is None:
        self.node.get_logger().error("annotated_img é None! DetectNet não retornou imagem.")
        return self.bases  # retorna sem publicar

    self.node.get_logger().info(f"Publicando imagem anotada, shapeannot: {annotated_img.shape}")
    out_msg = self.node.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
    self.node.annotated_pub.publish(out_msg)
    self.node.get_logger().warn("Imagem publicada em /ProcDroneImg")
    
    # 3. Processa as detecções para criar Targets
    for det in detections:
        if isinstance(det, dict) and 'force_vector' in det:
            fx = det['force_vector']['x']
            fy = det['force_vector']['y']
            
            # Estima posição global
            est_x = self.dronePos.X + (fy * self.dronePos.Z)
            est_y = self.dronePos.Y + (fx * self.dronePos.Z)

            est_z = 0.0 

            # Verifica duplicatas (raio de 1 metro)
            is_duplicate = False
            for target in self.bases:
                dist = math.sqrt((est_x - target.pos.X)**2 + (est_y - target.pos.Y)**2)
                if dist < 1.0:
                    is_duplicate = True
                    break
            
            # Adiciona novo Target se for único
            if not is_duplicate:
                new_id = len(self.bases) + 1
                new_target = Target(
                    id=new_id, 
                    pos=Position(est_x, est_y, est_z, 0),
                    visited=False
                )
                self.bases.append(new_target)
                self.node.get_logger().info(f"Novo Target #{new_id} em: X={est_x:.2f}, Y={est_y:.2f}")
            
    # return self.bases

    Targets = [
            Target(id=1, pos=Position((-9 + self.dronePos.X), (-1.6 + self.dronePos.Y), 0.0, 0)),
            Target(id=2, pos=Position((4 + self.dronePos.X), (-3.5 + self.dronePos.Y), 0.0, 0)),
            Target(id=3, pos=Position((0.0 + self.dronePos.X), (-5.0 + self.dronePos.Y), 0.0, 0)),
            Target(id=4, pos=Position((-4.5 + self.dronePos.X), (-0.5 + self.dronePos.Y), 0.0, 0)),
            Target(id=5, pos=Position((10.5 + self.dronePos.X), (-0.5 + self.dronePos.Y), 0.0, 0)),
        ]

    return Targets



def searchNearestBase(self, img):
    """
    Calcula o deslocamento relativo necessário, transformando as coordenadas
    da câmera (Body Frame) para o sistema global (NED) usado pela plataforma.
    """
    
    # 1. Atualiza e valida a posição do drone (proteção contra falha de leitura)
    self.dronePos = updateDronePos(self)
    if not self.dronePos:
        return Position(0.0, 0.0, 0.0, 0.0)

    # CORREÇÃO CRÍTICA 1: Altitude no sistema NED
    # Em NED, o eixo Z é positivo para BAIXO. Se o drone está a 10m de altura, Z = -10.
    # Multiplicar um vetor por um Z negativo inverte as direções calculadas.
    # Usamos abs() para garantir que a escala seja sempre positiva.
    altitude = abs(self.dronePos.Z)

    # CORREÇÃO CRÍTICA 2: Garantir que o Yaw é lido corretamente (assume-se radianos)
    # As funções math.cos e math.sin do Python exigem radianos. 
    current_yaw = getattr(self, 'droneYaw', 0.0) 

    _, detections = detect_single_frame(img)
    
    if not detections:
        return Position(0.0, 0.0, 0.0, 0.0)

    best_rel_pos = None
    min_dist_sq = float('inf')

    smooth_factor = 0.75  # Move apenas 40% do erro por iteração (ajuda a desacelerar)
    deadzone = 0.05      # Ignora erros menores que 5% (evita tremores finos)

    # Valores extraídos diretamente do SDF do Gazebo
    FOCAL_LENGTH = 205.47  # Calculado a partir de FOV 2.0 e Width 640
    REAL_BASE_SIZE_M = 1.0 # 1 metro

    for det in detections:
        if isinstance(det, dict) and 'force_vector' in det and 'width' in det:
            fx = det['force_vector']['x']
            fy = det['force_vector']['y']
            bbox_width_pixels = det['width']

            # Agora isso vai te dar a distância real (em metros) da lente até a base!
            dist_to_base = (REAL_BASE_SIZE_M * FOCAL_LENGTH) / bbox_width_pixels

            # 1. Aplica a Zona Morta (Deadzone)
            if abs(fx) < deadzone: fx = 0.0
            if abs(fy) < deadzone: fy = 0.0

            # 2. Coordenadas no referencial do DRONE (Body Frame) com Suavização
            # O eixo Y da imagem controla Frente/Trás (+X)
            # O eixo X da imagem controla Direita/Esquerda (+Y)
            dx_body = -(fy * dist_to_base) * smooth_factor
            dy_body = (fx * dist_to_base) * smooth_factor

            # 3. TRANSFORMAÇÃO PARA REFERENCIAL GLOBAL (NED)
            dx_ned = dx_body * math.cos(current_yaw) - dy_body * math.sin(current_yaw)
            dy_ned = dx_body * math.sin(current_yaw) + dy_body * math.cos(current_yaw)

            # 4. Verificação de visita (usando coordenadas globais NED)
            est_x_global = self.dronePos.X + dx_ned
            est_y_global = self.dronePos.Y + dy_ned

            if is_base_already_visited(self, est_x_global, est_y_global):
                continue

            dist_sq = dx_ned**2 + dy_ned**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_rel_pos = Position(dx_ned, dy_ned, 0.0, 0.0)

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

            annotated_img, detections = detect_single_frame(self.frame)
            out_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
            self.annotated_pub.publish(out_msg)

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
            
            # Agrupando os comandos de movimentação e estado que apenas retornam "True"
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
        {'trigger': 'Explore_to_Final', 'source': 'Explore', 'dest': 'Final', 'conditions': 'cond_Explore_Final'},

        {'trigger': 'SearchForBases_to_Explore', 'source': 'SearchForBases', 'dest': 'Explore', 'conditions': 'cond_SearchForBases_Explore', 'before': 'before_SearchForBases_Explore'},
        {'trigger': 'SearchForBases_to_GoToBase', 'source': 'SearchForBases', 'dest': 'GoToBase', 'conditions': 'cond_SearchForBases_GoToBase', 'before': 'before_SearchForBases_GoToBase'},

        {'trigger': 'GoToBase_to_ApproachToBase', 'source': 'GoToBase', 'dest': 'ApproachToBase'},

        {'trigger': 'ApproachToBase_to_ApproachToBase', 'source': 'ApproachToBase', 'dest': 'ApproachToBase', 'conditions': 'cond_ApproachToBase_ApproachToBase', 'before': 'before_ApproachToBase_ApproachToBase'},
        {'trigger': 'ApproachToBase_to_LandAndScore', 'source': 'ApproachToBase', 'dest': 'LandAndScore', 'conditions': 'cond_ApproachToBase_LandAndScore'},

        {'trigger': 'LandAndScore_to_TakeOff', 'source': 'LandAndScore', 'dest': 'TakeOff'},

        {'trigger': 'TakeOff_to_Explore', 'source': 'TakeOff', 'dest': 'Explore', 'conditions': 'cond_TakeOff_Explore'},
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
        self.visitedBases = 1
        self.bases: list[Target] = []
        self.defPos = [ Position(2 + self.homePos.X, 2 + self.homePos.Y,self.targetHeight, 0), 
                        Position(-4 + self.homePos.X,-4 + self.homePos.Y,self.targetHeight, 0),
                        Position(-6 + self.homePos.X,-6 + self.homePos.Y,self.targetHeight, 0)]
        
        
        # Targets = [
        #     Target(id=1, pos=Position((-9.50 + self.dronePos.X), (-1.0 + self.dronePos.Y), 0.0)),
        #     Target(id=2, pos=Position((4.5 + self.dronePos.X), (-3.0 + self.dronePos.Y), 0.0)),
        #     Target(id=3, pos=Position((0.0 + self.dronePos.X), (-5.0 + self.dronePos.Y), 0.0)),
        #     Target(id=4, pos=Position((-4.5 + self.dronePos.X), (-0.5 + self.dronePos.Y), 0.0)),
        #     Target(id=5, pos=Position((10.5 + self.dronePos.X), (-0.5 + self.dronePos.Y), 0.0)),
        # ]

        self.basePos = Position(0,0,0, 0)
        self.dronePos = Position(0,0,0, 0)
        self.distToTarget = 99999999.0

        self.MAX_ATTEMPT = 3
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
        return self.count <= self.MAX_ATTEMPT
    
    def cond_Explore_Final(self):
        return self.count > self.MAX_ATTEMPT

    def cond_SearchForBases_Explore(self):
        return len(self.bases) < 2 and self.count <= self.MAX_ATTEMPT
            
    def cond_SearchForBases_GoToBase(self):
        return len(self.bases) > 1
    
    def cond_ApproachToBase_ApproachToBase(self):
        return (self.distToTarget >= self.SAFE_DISTANCE)
    
    def cond_ApproachToBase_LandAndScore(self):
        return (self.distToTarget < self.SAFE_DISTANCE)
    
    def cond_TakeOff_Explore(self):
        return (self.visitedBases < 5)
    
    def cond_TakeOff_Final(self):
        return (self.visitedBases >= 5)
    
    ####################### Before Transitions ####################### 
    def before_Initial_Explore(self):
        self.count = 1
        self.visitedBases = 0

    def reset_trigger_image(self):
        self.trigger_image = False

    def before_SearchForBases_Explore(self):
        Request(self.node, f"homeAbsMove;{self.defPos[self.count].X};{self.defPos[self.count].Y};{self.defPos[self.count].Z}; 0")
        Wait()
        self.count = self.count + 1

    def before_SearchForBases_GoToBase(self):
        self.count = 1

    def before_ApproachToBase_ApproachToBase(self):
        print("Moving to new Position...")
        Request(self.node, f"relMove;{self.basePos.X};{self.basePos.Y};{self.basePos.Z}")
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
        target_global = self.bases[self.visitedBases].pos
        self.dronePos = updateDronePos(self)

        # offset relativo
        delta_x = target_global.X - self.dronePos.X
        delta_y = target_global.Y - self.dronePos.Y
        delta_z = 0

        Request(self.node, f"relMove;{delta_x};{delta_y};{delta_z}")
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
        
        # Atualiza posição atual antes de salvar
        self.dronePos = updateDronePos(self)
        
        # Salva a posição para não repetir
        self.visited_bases_coords.append((self.dronePos.X, self.dronePos.Y))
        
        # Comando de pouso via ROS
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

        #{'trigger': 'Phases_to_Final', 'source': 'Phases', 'dest': 'Final', 'before': ['before_Phases_Final', 'reset_trigger']} -> Multiplas ações antes da transição
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

        #connection_string = "tcp:127.0.0.1:5760"
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

    # Aguarda a FSM terminar
    try:
        fsm_thread.join() # O programa principal fica esperando a FSM acabar aqui
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")
    finally:
        # Quando a FSM termina (ou se der erro/Ctrl+C), nós derrubamos o ROS
        node.get_logger().info("Encerrando ROS 2...")
        if rclpy.ok():
            rclpy.shutdown() # Isso quebra o loop infinito do rclpy.spin()
        
        # Agora sim a thread do ROS consegue finalizar em paz
        ros_thread.join(timeout=2.0) 
        print("Programa encerrado!")

if __name__ == '__main__':
    main()
