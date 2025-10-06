# machine2_node.py
# pip install transitions graphviz

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import UInt8



class Machine2Node(Node):
    def __init__(self):
        super().__init__('machine2_node')
        self.get_logger().info("[M2] Node iniciado, criando máquina de estados...")
        self.publisher = self.create_publisher(UInt8, '/trigger_start', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        comando = input("Informe o valor a ser enviado: ")
        self.msg = UInt8()
        self.msg.data = int(comando)
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = Machine2Node()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
