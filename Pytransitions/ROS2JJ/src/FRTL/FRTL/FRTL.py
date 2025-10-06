import os, time, rclpy, threading
from transitions.extensions import GraphMachine
from rclpy.node import Node
from std_msgs.msg import UInt8
import auxFuncs

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
        self.home_pos = [0, 0, 0, 0]
        self.uav = None
        self.finished = False
        self.node = node
        self.start_time = time.time()

        self.connectionState = 0 
        self.phase = 0
        self.connectionTrys = 0
        self.targetHeight = 4

        self.trigger_start = False

        ####################### Draw State Machine ######################
        try:
            out_dir = 'Pytransitions/ROS2JJ/src/FRTL/FRTL/Data/'
            os.makedirs(out_dir, exist_ok=True)
            
            self.get_graph().draw(os.path.join(out_dir, 'FRTL.canon'), prog='dot')
            self.get_graph().draw(os.path.join(out_dir, 'FRTL.png'), prog='dot')

        except Exception as e:
            self.node.get_logger().error(f"Não foi possível gerar diagrama: {e}")

####################### Transition Conditions ####################### 
    def cond_Connect_Connect(self):
        if self.connectionState == 2 and self.connectionTrys < 3:
            return True
        return False
    
    def cond_Connect_Wait(self):
        if self.connectionState == 1:
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
        print("Returning to Launch")
        auxFuncs.set_mode(self.uav, 'RTL')
        while True:
            cond = auxFuncs.has_reached_position(self.uav, self.home_pos['lat'], self.home_pos['lon'], self.home_pos['alt'])
            time.sleep(1)
            if cond:
                print("Posição alvo alcançada!")
                break

####################### On_enter States #######################         
    def on_enter_Connect(self):
        self.node.get_logger().warn("\non_enter_Connect")
        self.uav = auxFuncs.connect_drone(self.connection_string)
        self.connectionState = auxFuncs.wait_for_heartbeat(self.uav)

    def on_enter_Wait(self):
        self.node.get_logger().warn("\non_enter_Wait")
        auxFuncs.set_home_to_current_position(self.uav)
        self.home_pos = auxFuncs.get_home_position(self.uav)
        print("Waiting for start...\n")

    def on_enter_StartEngines(self):
        self.node.get_logger().warn("\non_enter_StartEngines")
        auxFuncs.set_mode(self.uav, "GUIDED")
        auxFuncs.arm_drone(self.uav)
        time.sleep(1)

    def on_enter_TakeOff(self):
        self.node.get_logger().warn("\non_enter_TakeOff")
        auxFuncs.takeoff_relative(self.uav, self.targetHeight, self.home_pos['alt'])

    def on_enter_Phases(self):
        self.node.get_logger().warn("\non_enter_Phases")
        print("Going towards first point for 25 seconds ...")
        auxFuncs.move_to_relative(self.uav, self.start_time, -5, 5, 0, 0)
        text = "\nGoing to " + str(-35.361354) + "; "+ str(149.165218) + "; " + str(self.targetHeight) 
        time.sleep(10)

        print("Going towards second point for 25 seconds ...")
        auxFuncs.move_to_relative(self.uav, self.start_time, 5, -5, 0, 0)
        time.sleep(10)

    def on_enter_Final(self):
        self.node.get_logger().warn("\non_enter_Final")
        auxFuncs.close_connection(self.uav)
        self.finished = True

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
                self.node.get_logger().warn("Not Tock")

class FrtlNode(Node):
    def __init__(self):
        super().__init__('machine1_node')
        self.get_logger().info("Node iniciado, criando máquina de estados...")
        self.machine = FRTL(self)

        # subscriber para receber triggers
        self.create_subscription(UInt8, '/trigger_start', self.start_callback, 10)

    def start_callback(self, msg: UInt8):
        self.machine.trigger_start = True
        self.machine.phase = msg.data
        self.get_logger().warn(f"\n Command received: {self.machine.trigger_start}, in: {time.time()} ###")

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
