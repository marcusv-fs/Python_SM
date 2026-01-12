import os, time, rclpy, threading, math
from transitions.extensions import GraphMachine
from rclpy.node import Node
from std_msgs.msg import UInt8, String
from dataclasses import dataclass
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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

@dataclass
class Target:
    id: int
    pos: Position
    visited: bool = False

def Wait():
    global Wait_flag
    Wait_flag = False
    while Wait_flag == False:
        time.sleep(1)
    Wait_flag = False

def Request(node: Node, command : str):
    msg.data = command
    node.request_pub.publish(msg)

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

        self.targetHeight = TARGET_HEIGHT
        self.homePos = homePos
        self.finished = False
        self.node = node
        self.start_time = time.time()

        self.img = None
        self.trigger_image = False
        self.count = 0
        self.visitedBases = 0
        self.bases: list[Target] = []
        self.defPos = [ Position(1 + self.homePos.X,1 + self.homePos.Y,self.targetHeight), 
                        Position(2 + self.homePos.X,2 + self.homePos.Y,self.targetHeight),
                        Position(3 + self.homePos.X,3 + self.homePos.Y,self.targetHeight)]
        self.basePos = Position(0,0,0)
        self.dronePos = Position(0,0,0)
        self.distToTarget = 0.0

        self.MAX_ATTEMPT = 3
        self.SAFE_DISTANCE = 0.5

        ####################### Draw State Machine ######################
        try:
            out_dir = 'Pytransitions/FRTL/Data/'
            os.makedirs(out_dir, exist_ok=True)
            
            self.get_graph().draw(os.path.join(out_dir, 'Phase1.canon'), prog='dot')
            self.get_graph().draw(os.path.join(out_dir, 'Phase1.png'), prog='dot')

        except Exception as e:
            self.node.get_logger().error(f"Não foi possível gerar diagrama: {e}")

####################### Mission Functions #######################     
    def searchBases(self, img):
        if self.visitedBases < 1:
            Targets = [
                Target(id=1, pos=Position((-9.50 + self.dronePos.X), (-1.0 + self.dronePos.Y), 0.0)),
                Target(id=2, pos=Position((4.5 + self.dronePos.X), (-3.0 + self.dronePos.Y), 0.0)),
                Target(id=3, pos=Position((0.0 + self.dronePos.X), (-5.0 + self.dronePos.Y), 0.0)),
                Target(id=4, pos=Position((-4.5 + self.dronePos.X), (-0.5 + self.dronePos.Y), 0.0)),
                Target(id=5, pos=Position((10.5 + self.dronePos.X), (-0.5 + self.dronePos.Y), 0.0)),
            ]
            for base in Targets:
                self.bases.append(base)
        return self.bases
    
    def searchNearestBase(self, img):
        X = self.bases[self.visitedBases].pos.X - self.dronePos.X
        Y = self.bases[self.visitedBases].pos.Y - self.dronePos.Y
        Z = self.bases[self.visitedBases].pos.Z

        basePosError = Position(0 , 0, 0)

        print(f"dronePos: {self.dronePos}")
        print(f"basePos: {self.basePos}")
        print(f"basePosError: {basePosError}")

        return basePosError
    
    def updateDronePos(self):
        try:
            Request(self.node, f"getLocalPos")
            Wait()
            return self.dronePos

        except Exception as e:
            print(f"Erro ao atualizar a posição do drone: {e}")
            Request(self.node, f"setMode;EMERGENCY")
            exit(1)

    def calcDist(self, basePos: Position, dronePos: Position):
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

####################### Transition Conditions ####################### 
    def cond_Explore_SearchForBases(self):
        return self.count <= self.MAX_ATTEMPT
    
    def cond_Explore_Final(self):
        return self.count > self.MAX_ATTEMPT

    def cond_SearchForBases_Explore(self):
        return len(self.bases) == 0
            
    def cond_SearchForBases_GoToBase(self):
        return len(self.bases) != 0
    
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
        Request(self.node, f"relMove;{self.defPos[self.count].X};{self.defPos[self.count].Y};{self.defPos[self.count].Z}")
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
        self.bases = self.searchBases(self.img)

    def on_enter_GoToBase(self):
        self.node.get_logger().info("on_enter_GoToBase")
        self.basePos = self.bases[self.visitedBases].pos
        Request(self.node, f"relMove;{self.basePos.X};{self.basePos.Y};{self.basePos.Z}")
        Wait()

    def on_enter_ApproachToBase(self):
        self.node.get_logger().info("on_enter_ApproachToBase")

        while(self.trigger_image == False):
            time.sleep(0.1)
        self.trigger_image = False

        self.dronePos = self.updateDronePos()
        self.basePos = self.searchNearestBase(self.img)
        self.distToTarget = self.calcDist(self.basePos, self.dronePos)
        print({self.distToTarget})

    def on_enter_LandAndScore(self):
        self.node.get_logger().info("on_enter_LandAndScore")
        print("Landing...")
        Request(self.node, f"land")
        Wait()

        print("Update DronePos and markVisitedBases...")
        self.updateDronePos()
        self.markVisitedBases(self.dronePos)
        self.visitedBases = self.visitedBases + 1

    def on_enter_TakeOff(self):
        self.node.get_logger().info("on_enter_TakeOff")
        print("Changing to Guided...")
        Request(self.node, f"setMode;GUIDED")

        print("Arming...")
        Request(self.node, f"armUAV")

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
        self.homePos = Position(0,0,0)
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

###################### Mission Functions #######################
    def setHome(self):
        Request(self.node, f"setHome")
        Wait()
        return self.homePos

    def tryToConnect(self):
        Request(self.node, f"tryToConnect;{self.connection_string}")
        Wait()
        return self.isConnected

####################### On_enter States #######################         
    def on_enter_Connect(self):
        self.node.get_logger().info("on_enter_Connect")
        self.isConnected = self.tryToConnect()
        self.connectionTrys = self.connectionTrys + 1

    def on_enter_Wait(self):
        self.node.get_logger().info("on_enter_Wait")
        print("Waiting for start...\n")
        self.homePos = self.setHome()
        

    def on_enter_StartEngines(self):
        self.node.get_logger().info("on_enter_StartEngines")
        Request(self.node, f"setMode;GUIDED")
        Request(self.node, f"armUAV")
        

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

    def start_callback(self, msg: UInt8):
        self.machine.trigger_start = True
        self.machine.phase = msg.data
        self.get_logger().info(f"\n Command received: {self.machine.trigger_start}, in: {time.time()} ###")
        if(msg.data == 2):
            msg = String()
            msg.data = f"setMode;EMERGENCY"
            self.request_pub.publish(msg)
            exit(1)

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        try:
            self.machine.mission.img = self.frame
            self.machine.mission.trigger_image = True
        except:
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
            case "relTakeOff":
                if resp[1] == "True":
                    Wait_flag = True
                else:
                    error = True
            case "relMove":
                if resp[1] == "True":
                    Wait_flag = True
                else:
                    error = True
            case "land":
                if resp[1] == "True":
                    Wait_flag = True
                else:
                    error = True
            case "gpsMove":
                if resp[1] == "True":
                    Wait_flag = True
                else:
                    error = True
            case "setHome":
                if resp[1] == "True":
                    Wait_flag = True
                else:
                    error = True
        if error:
            self.get_logger().error("; ".join(resp))

######################## MAIN ########################
def ros_spin_thread(node):
    """Thread dedicada ao ROS2 spin."""
    rclpy.spin(node)

def machine_thread(machine):
    """Thread dedicada à máquina de estados."""
    machine.run()

def main(args=None):
    rclpy.init(args=args)
    node = FrtlNode()

    # Cria threads
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    fsm_thread = threading.Thread(target=machine_thread, args=(node.machine,), daemon=True)

    # Inicia as threads
    ros_thread.start()
    fsm_thread.start()
    time.sleep(5)

    # Aguarda a FSM terminar
    fsm_thread.join()

    # Quando FSM termina, encerra ROS
    node.get_logger().info("Encerrando ROS 2...")
    rclpy.shutdown()
    ros_thread.join()
    print("Programa encerrado!")

if __name__ == '__main__':
    main()
