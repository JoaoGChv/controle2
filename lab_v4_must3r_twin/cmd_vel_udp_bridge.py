"""
Ponte /cmd_vel -> UDP. Corre no PYTHON DO SISTEMA (ROS2 Humble, py3.10 — onde o rclpy funciona),
NÃO no Python do Isaac (py3.11, incompatível com o rclpy do Humble). Reencaminha cada Twist de
/cmd_vel para um socket UDP local que o run_isaac_teleop.py lê.

Uso:
  source /opt/ros/humble/setup.bash
  python3 cmd_vel_udp_bridge.py            # (usa python3 do sistema, NÃO o do Isaac)
Opções: --host 127.0.0.1 --port 9091
"""
import socket, sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HOST = sys.argv[sys.argv.index("--host")+1] if "--host" in sys.argv else "127.0.0.1"
PORT = int(sys.argv[sys.argv.index("--port")+1]) if "--port" in sys.argv else 9091


class Bridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_udp_bridge")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (HOST, PORT)
        self.n = 0
        self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.get_logger().info(f"a reencaminhar /cmd_vel -> UDP {HOST}:{PORT}")

    def cb(self, msg):
        v, w = float(msg.linear.x), float(msg.angular.z)
        self.sock.sendto(f"{v:.4f},{w:.4f}".encode(), self.addr)
        self.n += 1
        if self.n % 50 == 0:
            self.get_logger().info(f"cmd_vel #{self.n}: v={v:.2f} w={w:.2f}")


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
