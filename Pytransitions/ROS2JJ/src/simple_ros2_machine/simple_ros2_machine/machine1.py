# pip install transitions
# pip install graphviz

import os, time, rclpy
from transitions.extensions import GraphMachine
from rclpy.node import Node
from std_msgs.msg import Int32

class Machine1(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial','Operational', 'Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_Operational', 'source': 'Initial', 'dest': 'Operational'},
        {'trigger': 'Operational_to_Final', 'source': 'Operational', 'dest': 'Final', 'conditions': 'c_Final'},
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
        self.stop_command = 0 
        self.finished = False
        self.node = node

        ####################### Draw State Machine ######################
        try:
            out_dir = 'Pytransitions/ROS2JJ/src/simple_ros2_machine/Data/'
            os.makedirs(out_dir, exist_ok=True)
            
            self.get_graph().draw(os.path.join(out_dir, 'Machine1.canon'), prog='dot')
            self.get_graph().draw(os.path.join(out_dir, 'Machine1.png'), prog='dot') 

        except Exception as e:
            self.node.get_logger().warn(f"Não foi possível gerar diagrama: {e}")

    ##############################Util Functions##############################
    def move(self):
        self.node.get_logger().info("Moving...")

####################### Transition Conditions ####################### 
    def c_Final(self):
        print(f"stop_command value = {self.stop_command == 1}")
        return self.stop_command == 1

####################### Before Transitions ####################### 

####################### On_enter States #######################      
    def on_enter_Initial(self):
        self.node.get_logger().info("Entrou em Initial")

    def on_enter_Operational(self):
        self.node.get_logger().info("Entrou em Operational")
        self.move()
        time.sleep(0.1)

    def on_enter_Final(self):
        self.node.get_logger().info("Entrou em Final. Máquina finalizada.")
        self.finished = True

    # ####################### Main Loop #######################
    # def run(self):
    #     self.cycle = 0
    #     while True:
    #         print("\n/////////////////////// Cycle: " + str(self.cycle) + " ///////////////////////\n")
    #         cond = 0
            
    #         print("Now my current state is " + self.state)

    #         if(self.state == 'Initial'):
    #             self.Initial_to_Operational()

    #         if(self.state == 'Operational'):
    #             self.Operational_to_Final()

    #         if(self.state == 'Final'):
    #             print("Finished.")   
            
    #         self.cycle += 1
    #         time.sleep(0.5)


class MachineNode(Node):
    def __init__(self):
        super().__init__('machine1_node')
        self.get_logger().info("Node iniciado, criando máquina de estados...")
        self.machine = Machine1(self)

        #subscriber para receber comandos
        self.create_subscription(Int32, '/machine1_command', self.command_callback, 10)

        #timer para executar ciclos periódicos da máquina
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.cycle = 0

    def command_callback(self, msg: Int32):
        self.machine.stop_command = msg.data
        self.get_logger().warn(f"\n Command received in: {time.time()} ###")
        self.get_logger().info(f"Comando recebido: {self.machine.stop_command}")

    def timer_callback(self):
        if self.machine.finished:
            self.get_logger().info("Encerrando ROS 2...")
            self.timer.cancel()
            rclpy.shutdown()
            return
        
        self.get_logger().info(f"\n ### Cycle {self.cycle} ###")
        self.get_logger().info(f"Estado atual: {self.machine.state}")

        if self.machine.state == 'Initial':
            self.machine.Initial_to_Operational()

        elif self.machine.state == 'Operational':
            self.machine.Operational_to_Final()

        elif self.machine.state == 'Final':
            self.get_logger().info("Finished.")
            self.machine.finished = True

        self.cycle += 1


def main(args=None):
    rclpy.init(args=args)
    node = MachineNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

# Comando para enviar a mensagem no tópico 
# ros2 topic pub /machine1_command std_msgs/msg/Int32 "{data: 1}"
