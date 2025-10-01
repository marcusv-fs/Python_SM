# pip install transitions
# pip install graphviz
# pip install pyyaml
# pip install numpy

import os, time, rclpy, threading
from transitions.extensions import GraphMachine
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8

class Machine1(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial','Phase1', 'Phase2', 'Phase3', 'Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_Phase1', 'source': 'Initial', 'dest': 'Phase1'},
        {'trigger': 'Phase1_to_Phase2', 'source': 'Phase1', 'dest': 'Phase2', 'conditions': 'cond_Phase1_Phase2', 'before': 'reset_trigger_P1P2'},
        {'trigger': 'Phase2_to_Phase1', 'source': 'Phase2', 'dest': 'Phase1', 'conditions': 'cond_Phase2_Phase1', 'before': 'reset_trigger_P2P1'},
        {'trigger': 'Phase2_to_Phase3', 'source': 'Phase2', 'dest': 'Phase3', 'conditions': 'cond_Phase2_Phase3',  'before': 'reset_trigger_P2P3'},
        {'trigger': 'Phase3_to_Final', 'source': 'Phase3', 'dest': 'Final', 'conditions': 'cond_Phase3_Final',  'before': 'reset_trigger_P3Fn'},
    ]
    
####################### Init and util Functions ####################### 
    def __init__(self, node: Node):
        super().__init__(
            model=self, 
            states=Machine1.states,
            transitions=Machine1.transitions,
            initial='Initial', 
            auto_transitions=False,
            show_conditions=True,
            show_state_attributes=True,
            name="Machine1"
        )
        self.trigger_P1P2 = False
        self.trigger_P2P1 = False
        self.trigger_P2P3 = False
        self.trigger_P3Fn = False
       
        self.received_value = 0

        self.finished = False
        self.node = node

        ####################### Draw State Machine ######################
        try:
            out_dir = 'Pytransitions/ROS2JJ/src/simple_ros2_machine/Data/'
            os.makedirs(out_dir, exist_ok=True)
            
            self.get_graph().draw(os.path.join(out_dir, 'Machine1_t.canon'), prog='dot')
            self.get_graph().draw(os.path.join(out_dir, 'Machine1_t.png'), prog='dot')

        except Exception as e:
            self.node.get_logger().warn(f"Não foi possível gerar diagrama: {e}")

    ############################## Util Functions ##############################
    def move(self):
        self.node.get_logger().info("Moving...")

####################### Transition Conditions ####################### 
    def cond_Phase1_Phase2(self):
        return self.trigger_P1P2
    
    def cond_Phase2_Phase1(self):
        return self.trigger_P2P1
    
    def cond_Phase2_Phase3(self): 
        return self.trigger_P2P3
    
    def cond_Phase3_Final(self):
        condition = self.received_value > 5
        return self.trigger_P3Fn and condition

####################### Before Transitions ####################### 
    def reset_trigger_P1P2(self):
        self.trigger_P1P2 = False

    def reset_trigger_P2P1(self):
        self.trigger_P2P1 = False

    def reset_trigger_P2P3(self):
        self.trigger_P2P3 = False

    def reset_trigger_P3Fn(self):
        self.trigger_P3Fn = False

####################### On_enter States #######################      
    def on_enter_Initial(self):
        self.node.get_logger().info("Entrou em Initial")

    def on_enter_Phase1(self):
        self.node.get_logger().info("Entrou em Phase1")
        self.move()
        time.sleep(0.1)

    def on_enter_Phase2(self):
        self.node.get_logger().info("Entrou em Phase2")
        self.move()
        time.sleep(0.1)

    def on_enter_Phase3(self):
        self.node.get_logger().info("Entrou em Phase3")
        self.move()
        time.sleep(0.1)

    def on_enter_Final(self):
        self.node.get_logger().info("Entrou em Final. Máquina finalizada.")
        self.finished = True

    ####################### Main Loop #######################
    def run(self):
        while not self.finished:
            self.tock = 1
            for transition in self.transitions:
                if self.state in str(transition.get("source")):
                    if self.may_trigger(transition.get("trigger")):
                        self.tock = 0
                        self.trigger(transition.get("trigger"))
            if self.tock == 1:
                print("tock -> ")
                time.sleep(0.5) 
            else:
                print ("--------------NOOOOOOT tock")

            print(f"Current State: {self.state}")


class MachineNode(Node):
    def __init__(self):
        super().__init__('machine1_node')
        self.get_logger().info("Node iniciado, criando máquina de estados...")
        self.machine = Machine1(self)

        # subscriber para receber comandos
        self.create_subscription(Bool, '/trigger_P1P2', self.P1P2_callback, 10)
        self.create_subscription(Bool, '/trigger_P2P1', self.P2P1_callback, 10)
        self.create_subscription(Bool, '/trigger_P2P3', self.P2P3_callback, 10)
        self.create_subscription(UInt8, '/trigger_P3Fn', self.P3Fn_callback, 10)

    def P1P2_callback(self, msg: Bool):
        self.machine.trigger_P1P2 = True
        self.get_logger().warn(f"\n Command received: {self.machine.trigger_P1P2}, in: {time.time()} ###")

    def P2P1_callback(self, msg: Bool):
        self.machine.trigger_P2P1 = True
        self.get_logger().warn(f"\n Command received: {self.machine.trigger_P2P1}, in: {time.time()} ###")

    def P2P3_callback(self, msg: Bool):
        self.machine.trigger_P2P3 = msg.data
        self.get_logger().warn(f"\n Command received: {self.machine.trigger_P2P3}, in: {time.time()} ###")

    def P3Fn_callback(self, msg: UInt8):
        self.machine.trigger_P3Fn = True
        self.machine.received_value = msg.data
        self.get_logger().warn(f"\n Command received: {self.machine.trigger_P3Fn}, value received: {msg.data}, in: {time.time()} ###")


######################## MAIN ########################
def ros_spin_thread(node):
    """Thread dedicada ao ROS2 spin."""
    rclpy.spin(node)


def machine_thread(machine):
    """Thread dedicada à máquina de estados."""
    machine.run()


def main(args=None):
    rclpy.init(args=args)
    node = MachineNode()

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

# Comando para enviar a mensagem no tópico 
# ros2 topic pub /machine1_command std_msgs/msg/Bool "{data: 1}"