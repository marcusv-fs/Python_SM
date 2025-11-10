import os, time, rclpy, threading, math
from transitions.extensions import GraphMachine
from rclpy.node import Node
from std_msgs.msg import UInt8, String
import Platform
from dataclasses import dataclass

TARGET_HEIGHT = 3
resp = ['']
msg = String()
global wait
wait = False

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
    
class Phase1(GraphMachine):
    ####################### States Declaration #######################   
    states = ['Initial', 'Explore', 'SearchForBases', 'GoToBase', 'ApproachToBase', 'LandAndScore', 'TakeOff', 'Final']

    ####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_Explore', 'source': 'Initial', 'dest': 'Explore', 'before': 'before_Initial_Explore'},

        {'trigger': 'Explore_to_SearchForBases', 'source': 'Explore', 'dest': 'SearchForBases', 'conditions': 'cond_Explore_SearchForBases'},
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

    def __init__(self, node: Node, uav, homePos):    
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

        self.uav = uav
        self.targetHeight = TARGET_HEIGHT
        self.homePos = homePos
        self.finished = False
        self.node = node
        self.start_time = time.time()

        self.count = 0
        self.visitedBases = 0
        self.bases: list[Target] = []
        self.defPos = [Position(1,1,4), Position(2,2,4), Position(3,3,4)]
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
    def searchForBases(self):
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
    
    def searchNearestBase(self):
        X = self.bases[self.visitedBases].pos.X - self.dronePos.X
        Y = self.bases[self.visitedBases].pos.Y - self.dronePos.Y
        Z = self.bases[self.visitedBases].pos.Z

        basePosError = Position(0 , 0, 0)

        print(f"dronePos: {self.dronePos}")
        print(f"basePos: {self.basePos}")
        print(f"basePosError: {basePosError}")

        self.basePos = basePosError
    
    def updateDronePos(self):
        global wait

        try:
            msg.data = f"getLocalPos"
            self.node.request_pub.publish(msg)
            wait = False

            while wait == False:
                time.sleep(1)
            
            wait = False

        except Exception as e:
            print(f"Erro ao atualizar a posição do drone: {e}")
            msg.data = f"setMode;EMERGENCY"
            self.node.request_pub.publish(msg)
            wait = False
            exit(1)

    def calcDist(self, basePos: Position, dronePos: Position):
        return math.sqrt(
        (basePos.X) ** 2 +
        (basePos.Y) ** 2
    )

    def calcDist2(self, basePos: Position, dronePos: Position):
        return math.sqrt(
        (basePos.X - dronePos.X) ** 2 +
        (basePos.Y - dronePos.Y) ** 2
    )

    def markVisitedBases(self, current_position: Position):
        closestBaseID = 0
        dist = 9999999
        for base in self.bases:
            auxDist = self.calcDist2(base.pos, current_position)
            if dist > auxDist:
                dist = auxDist
                closestBaseID = base.id
            
        for base in self.bases:
            if closestBaseID == base.id:
                base.visited = True

####################### Transition Conditions ####################### 
    def cond_Explore_SearchForBases(self):
        if self.count <= self.MAX_ATTEMPT:
            return True
        return False
    
    def cond_Explore_Final(self):
        if self.count > self.MAX_ATTEMPT:
            return True
        return False

    def cond_SearchForBases_Explore(self):
        if len(self.bases) == 0:
            return True
        return False
            
    def cond_SearchForBases_GoToBase(self):
        if len(self.bases) != 0:
            return True
        return False
    
    def cond_ApproachToBase_ApproachToBase(self):
        if (self.distToTarget >= self.SAFE_DISTANCE):
            return True
        return False
    
    def cond_ApproachToBase_LandAndScore(self):
        if (self.distToTarget < self.SAFE_DISTANCE):
            return True
        return False
    
    def cond_TakeOff_Explore(self):
        if (self.visitedBases < 5):
            return True
        return False
    
    def cond_TakeOff_Final(self):
        if (self.visitedBases >= 5):
            return True
        return False
    
    ####################### Before Transitions ####################### 
    def before_Initial_Explore(self):
        self.count = 1
        self.visitedBases = 0

    def before_SearchForBases_Explore(self):
        self.count += 1
        time.sleep(1)

    def before_SearchForBases_GoToBase(self):
        self.count = 1

    def before_ApproachToBase_ApproachToBase(self):
        print("Moving to new Position...")

        msg.data = f"relMove;{self.basePos.X};{self.basePos.Y};{self.basePos.Z}"
        self.node.request_pub.publish(msg)

        global wait
        while wait == False:
            time.sleep(1)

    ####################### On_enter States #######################         
    def on_enter_Explore(self):
        pass

    def on_enter_SearchForBases(self):
        self.node.get_logger().info("on_enter_SearchForBases")
        self.searchForBases()

    def on_enter_GoToBase(self):
        self.node.get_logger().info("on_enter_GoToBase")
        self.basePos = self.bases[self.visitedBases].pos

        msg.data = f"relMove;{self.basePos.X};{self.basePos.Y};{self.basePos.Z}"
        self.node.request_pub.publish(msg)

        global wait
        while wait == False:
            time.sleep(1)


    def on_enter_ApproachToBase(self):
        self.node.get_logger().info("on_enter_ApproachToBase")
        self.updateDronePos()
        self.searchNearestBase()
        self.distToTarget = self.calcDist(self.basePos, self.dronePos)
        print({self.distToTarget})

    def on_enter_LandAndScore(self):
        global wait
        self.node.get_logger().info("on_enter_LandAndScore")
        print("Landing...")
        msg.data = f"landAndDisarm"
        self.node.request_pub.publish(msg)
        wait = False
        while wait == False:
            time.sleep(1)

        print("Update DronePos and markVisitedBases...")
        self.updateDronePos()
        self.markVisitedBases(self.dronePos)
        self.visitedBases += 1

    def on_enter_TakeOff(self):
        global wait
        self.node.get_logger().info("on_enter_TakeOff")
        print("Changing to Guided...")
        msg.data = f"setMode;GUIDED"
        self.node.request_pub.publish(msg)

        time.sleep(1)
        print("Arming...")
        msg.data = f"armUAV"
        self.node.request_pub.publish(msg)

        print("TakingOff...")
        msg.data = f"relTakeOff;{self.targetHeight};{self.homePos.Z}"
        self.node.request_pub.publish(msg)
        wait = False
        while wait == False:
            time.sleep(1)

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
        {'trigger': 'Connect_to_Connect', 'source': 'Connect', 'dest': 'Connect', 'conditions': 'cond_Connect_Connect', 'before': 'before_Connect_Connect'},
        {'trigger': 'Connect_to_Wait', 'source': 'Connect', 'dest': 'Wait', 'conditions': 'cond_Connect_Wait', 'before': 'reset_trigger_start'},
        {'trigger': 'Wait_to_StartEngines', 'source': 'Wait', 'dest': 'StartEngines', 'conditions': 'cond_Wait_StartEngines', 'before': 'reset_trigger_start'},
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

        #connection_string = "tcp:127.0.0.1:5760"
        self.connection_string = "udp:127.0.0.1:14551"
        self.uav = None
        self.finished = False
        self.node = node
        self.start_time = time.time()


        self.isConnected = False 
        self.phase = 0
        self.connectionTrys = 0
        self.homePos = Position(0,0,0)
        self.altitude = 0
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
    def cond_Connect_Connect(self):
        if self.isConnected == False and self.connectionTrys < 3:
            return True
        return False
    
    def cond_Connect_Wait(self):
        if self.isConnected == True:
            return True
        return False
    
    def cond_Wait_StartEngines(self):
        if self.trigger_start:
            return True
        return False

####################### Before Transitions ####################### 
    def reset_trigger_start(self):
        self.trigger_start = False

    def before_Connect_Connect(self):
        self.connectionTrys = self.connectionTrys + 1

    def before_Phases_Final(self):
        global wait
        print("Returning to Launch")
        wait = False
        msg.data = f"gpsMove;{self.homePos.X};{self.homePos.Y};{self.targetHeight}"
        self.node.request_pub.publish(msg)
        while wait == False:
            time.sleep(1)

        wait = False
        msg.data = f"landAndDisarm"
        self.node.request_pub.publish(msg)
        while wait == False:
            time.sleep(1)
        
        

####################### On_enter States #######################         
    def on_enter_Connect(self):
        global wait

        self.node.get_logger().info("on_enter_Connect")

        msg.data = f"tryToConnect;{self.connection_string}"
        self.node.request_pub.publish(msg)
        wait = False

        msg.data = f"checkConnection"
        self.node.request_pub.publish(msg)
        wait = False

        while wait == False:
            time.sleep(1) 
        
        wait = True

    def on_enter_Wait(self):
        global wait
        self.node.get_logger().info("on_enter_Wait")
        msg.data = f"setHomeHere"
        self.node.request_pub.publish(msg)
        wait = False

        while wait == False:
            time.sleep(1) 
        
        wait = True


        print("Waiting for start...\n")

    def on_enter_StartEngines(self):
        global wait
        self.node.get_logger().info("on_enter_StartEngines")
        msg.data = f"setMode;GUIDED"
        self.node.request_pub.publish(msg)
        wait = False

        msg.data = f"armUAV"
        self.node.request_pub.publish(msg)
        wait = False

        time.sleep(5)

    def on_enter_TakeOff(self):
        global wait
        self.node.get_logger().info("on_enter_TakeOff")

        msg.data = f"relTakeOff;{self.targetHeight};{self.homePos.Z}"
        self.node.request_pub.publish(msg)
        wait = False

        while wait == False:
            time.sleep(1)

        

    def on_enter_Phases(self):
        global wait
        self.node.get_logger().info("on_enter_Phases")
        if self.phase == 1:
            self.mission = Phase1(self.node, self.uav, self.homePos)
            self.mission.run()

    def on_enter_Final(self):
        global wait
        self.node.get_logger().info("on_enter_Final")
        wait = False
        msg.data = f"closeConnection"
        self.node.request_pub.publish(msg)
        time.sleep(5)
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

        # subscriber para receber triggers
        self.create_subscription(UInt8, '/trigger_start', self.start_callback, 10)
        
        # subscriber para receber comandos
        self.create_subscription(String, '/response', self.response_callback, 10)

        # publisher para requisitar ações da plataforma
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

    def response_callback(self, msg: String):
        global wait
        resp = msg.data.split(';')
        self.get_logger().warn(str(resp))
        error = False

        match resp[0]:
            case "getLocalPos":
                if resp[1] == "True":
                    self.machine.mission.dronePos.X = float(resp[2])
                    self.machine.mission.dronePos.Y = float(resp[3])
                    self.machine.mission.dronePos.Z = float(resp[4])
                    wait = True
                else:
                    error = True
            case "setHomeHere":
                if resp[1] == "True":
                    self.machine.homePos.X = float(resp[2])
                    self.machine.homePos.Y = float(resp[3])
                    self.machine.homePos.Z = float(resp[4])
                    wait = True
                else:
                    error = True
            case "checkConnection":
                if resp[1] == "True":
                    self.machine.isConnected = True
                    wait = True
                else:
                    error = True
            case "relTakeOff":
                if resp[1] == "True":
                    wait = True
                else:
                    error = True
            case "relMove":
                if resp[1] == "True":
                    wait = True
                else:
                    error = True
            case "landAndDisarm":
                if resp[1] == "True":
                    wait = True
                else:
                    error = True
            case "gpsMove":
                if resp[1] == "True":
                    wait = True
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

    # Aguarda a FSM terminar
    fsm_thread.join()

    # Quando FSM termina, encerra ROS
    node.get_logger().info("Encerrando ROS 2...")
    rclpy.shutdown()
    ros_thread.join()
    print("Programa encerrado!")

if __name__ == '__main__':
    main()
