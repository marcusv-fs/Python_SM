# machine2_node.py
# pip install transitions graphviz

import os, time, rclpy
from transitions.extensions import GraphMachine
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import UInt8


class Machine2(GraphMachine):
####################### States Declaration #######################   
    states = ['Initial', 'Waiting', 'Final']

####################### Transitions Statement  #######################  
    transitions = [
        {'trigger': 'Initial_to_Waiting', 'source': 'Initial', 'dest': 'Waiting'},
        {'trigger': 'Waiting_to_Final', 'source': 'Waiting', 'dest': 'Final', 'conditions': 'c_Final'},
    ]

####################### Init and util Functions ####################### 
    def __init__(self, node: Node):
        super().__init__(
            model=self,
            states=Machine2.states,
            transitions=Machine2.transitions,
            initial='Initial', 
            auto_transitions=False,
            show_conditions=True,
            show_state_attributes=True,
            name="Machine2"
        )
        self.node = node
        self.finished = False
        self.value = 0
        self.publisher1 = self.node.create_publisher(Bool, '/trigger_P1P2', 10)
        self.publisher2 = self.node.create_publisher(Bool, '/trigger_P2P1', 10)
        self.publisher3 = self.node.create_publisher(Bool, '/trigger_P2P3', 10)
        self.publisher4 = self.node.create_publisher(UInt8, '/trigger_P3Fn', 10)
        self.msg = Bool()
        self.msg4 = UInt8()

        ####################### Draw State Machine ######################
        try:
            out_dir = 'Pytransitions/ROS2JJ/src/simple_ros2_machine/Data/'
            os.makedirs(out_dir, exist_ok=True)
            self.get_graph().draw(os.path.join(out_dir, 'Machine2.canon'), prog='dot')
            self.get_graph().draw(os.path.join(out_dir, 'Machine2.png'), prog='dot')
        except Exception as e:
            self.node.get_logger().warn(f"[M2] Não foi possível gerar diagrama: {e}")

####################### Transition Conditions ####################### 
    def c_Final(self):
        print(f"[M2] End condition? value={self.value}")
        return self.value == 5

####################### On_enter States #######################    
    def on_enter_Initial(self):
        self.node.get_logger().info("[M2] Entrou em Initial")

    def on_enter_Waiting(self):
        self.node.get_logger().info("[M2] Aguardando comando no console...")
        try:
            user_input = input("Digite 1 para trigger 1, 2 para trigger 2, ..., 4 para trigger final e 5 para finalizar: ")
            self.value = int(user_input.strip())
            
            self.msg.data = True

            match(self.value):
                case 1:
                    self.publisher1.publish(self.msg)
                case 2:
                    self.publisher2.publish(self.msg)
                case 3:
                    self.publisher3.publish(self.msg)
                case 4:
                    user_input = input("Digite o valor que quer enviar: (> 5 finaliza machine 1) ")
                    self.value2 = int(user_input.strip())
                    self.msg4.data = self.value2
                    self.publisher4.publish(self.msg4)
                case _:
                    self.node.get_logger().error("Nenhum valor publicado")
            now = time.time()
            self.node.get_logger().warn(f"[M2] Publicado {self.value} em /machine1_command at {now}")
        except Exception as e:
            self.node.get_logger().error(f"[M2] Erro ao ler entrada: {e}")

    def on_enter_Final(self):
        self.node.get_logger().info("[M2] Entrou em Final. Máquina finalizada.")
        self.finished = True
        self.node.get_logger().info("[M2] Encerrando ROS 2...")
        rclpy.shutdown()


class Machine2Node(Node):
    def __init__(self):
        super().__init__('machine2_node')
        self.get_logger().info("[M2] Node iniciado, criando máquina de estados...")
        self.machine = Machine2(self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.cycle = 0

    def timer_callback(self):
        self.get_logger().info(f"\n[M2] ### Cycle {self.cycle} ###")
        self.get_logger().info(f"[M2] Estado atual: {self.machine.state}")

        if self.machine.state == 'Initial':
            self.machine.Initial_to_Waiting()

        elif self.machine.state == 'Waiting':
            self.machine.on_enter_Waiting()
            self.machine.Waiting_to_Final()

        elif self.machine.state == 'Final':
            self.get_logger().info("[M2] Finalizado.")
            self.machine.finished = True

        self.cycle += 1


def main(args=None):
    rclpy.init(args=args)
    node = Machine2Node()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
