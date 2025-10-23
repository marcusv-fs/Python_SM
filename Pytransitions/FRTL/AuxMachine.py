#!/usr/bin/env python3
import sys
sys.path.append('/usr/lib/python3/dist-packages')
import threading
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image as RosImage
import numpy as np
import gz.transport13 as gz
from gz.msgs10.image_pb2 import Image as GzImage

# ---------- ROS2 publisher da câmera do Gazebo ----------
class GazeboToROS2(Node):
    def __init__(self, gz_topic, ros_topic):
        super().__init__('gazebo_to_ros2')
        self.publisher_ = self.create_publisher(RosImage, ros_topic, 10)

        self.gz_node = gz.Node()
        if not self.gz_node.subscribe(GzImage, gz_topic, self.gz_callback):
            self.get_logger().error(f"Falha ao assinar o tópico Gazebo: {gz_topic}")
        else:
            self.get_logger().info(f"Inscrito no tópico Gazebo: {gz_topic}")

    def gz_callback(self, img_msg):
        width = img_msg.width
        height = img_msg.height
        data = np.frombuffer(img_msg.data, dtype=np.uint8)

        try:
            frame = data.reshape((height, width, 3))
        except ValueError:
            frame = data.reshape((height, width))

        ros_msg = RosImage()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = "camera_link"
        ros_msg.height = height
        ros_msg.width = width
        ros_msg.encoding = "rgb8" if len(frame.shape) == 3 else "mono8"
        ros_msg.is_bigendian = 0
        ros_msg.step = width * (3 if ros_msg.encoding == "rgb8" else 1)
        ros_msg.data = frame.tobytes()

        self.publisher_.publish(ros_msg)

    def spin_gz(self):
        try:
            while rclpy.ok():
                time.sleep(0.01)  # mantém o loop do Gazebo vivo
        except KeyboardInterrupt:
            pass

# ---------- Máquina de estados ----------
class Machine2Node(Node):
    def __init__(self):
        super().__init__('machine2_node')
        self.get_logger().info("[M2] Node iniciado, criando máquina de estados...")
        self.publisher = self.create_publisher(UInt8, '/trigger_start', 10)

    def send_command(self, value: int):
        msg = UInt8()
        msg.data = value
        self.publisher.publish(msg)
        self.get_logger().info(f"Comando enviado: {value}")

# ---------- Threads ----------
def camera_thread():
    gz_topic = "/world/FRTL_World/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
    ros_topic = "/camera/image_raw"

    node = GazeboToROS2(gz_topic, ros_topic)
    node.spin_gz()
    node.destroy_node()

def machine2_thread(node):
    try:
        while rclpy.ok():
            comando = input("Informe o valor a ser enviado: ")
            node.send_command(int(comando))
    except KeyboardInterrupt:
        pass

# ---------- Main ----------
def main():
    rclpy.init()

    # Máquina de estados
    machine_node = Machine2Node()

    # Threads
    t_camera = threading.Thread(target=camera_thread, daemon=True)
    t_machine = threading.Thread(target=machine2_thread, args=(machine_node,), daemon=True)

    t_camera.start()
    time.sleep(1)
    t_machine.start()

    t_camera.join()
    t_machine.join()

    try:
        # mantém o ROS2 ativo
        rclpy.spin(machine_node)
    except KeyboardInterrupt:
        pass
    finally:
        machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
