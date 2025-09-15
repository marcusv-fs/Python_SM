# machine2_node.py
# pip install transitions graphviz

import os
import rclpy
from transitions.extensions import GraphMachine
from rclpy.node import Node
from std_msgs.msg import Int32

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
        self.publisher = self.node.create_publisher(Int32, '/machine1_command', 10)
        self.value = 0


        ####################### Draw State Machine ######################
        try:
            out_dir = 'Pytransitions/ROS2JJ/src/simple_ros2_machine/Data/'
            os.makedirs(out_dir, exist_ok=True)
            
            self.get_graph().draw(os.path.join(out_dir, 'Machine2.canon'), prog='dot')
            self.get_graph().draw(os.path.join(out_dir, 'Machine2.png'), prog='dot')

        except Exception as e:
            self.node.get_logger().warn(f"Não foi possível gerar diagrama: {e}")

############################## Util Functions ##############################

####################### Transition Conditions ####################### 
    def c_Final(self):
        print(f"End condition? = {self.value == 1}")
        return self.value == 1


####################### Before Transitions ####################### 

####################### On_enter States #######################    

    def on_enter_Initial(self):
        self.node.get_logger().info("[M2] Entrou em Initial")

    def on_enter_Waiting(self):
        self.node.get_logger().info("[M2] Esperando comando no console...")
        try:
            user_input = input("Digite 1 para parar Machine1 ou 0 para continuar: ")
            value = int(user_input.strip())
            msg = Int32()
            msg.data = value
            self.publisher.publish(msg)
            self.node.get_logger().info(f"[M2] Publicado {value} em /machine1_command")
        except Exception as e:
            self.node.get_logger().error(f"[M2] Erro ao ler entrada: {e}")

    def on_enter_Final(self):
        self.node.get_logger().info("[M2] Entrou em Final. Máquina finalizada.")
        self.finished = True


class Machine2Node(Node):
    def __init__(self):
        super().__init__('machine2_node')
        self.get_logger().info("Node Machine2 iniciado.")
        self.machine = Machine2(self)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        if self.machine.finished:
            self.get_logger().info("[M2] Final detectado, encerrando node.")
            rclpy.shutdown()
            return

        if self.machine.state == 'Initial':
            self.machine.Initial_to_Waiting()
        elif self.machine.state == 'Waiting':
            self.machine.Waiting_to_Final()
            self.machine.on_enter_Waiting()
        elif self.machine.state == 'Final':
            self.get_logger().info("[M2] Finalizado.")


def main(args=None):
    rclpy.init(args=args)
    node = Machine2Node()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
